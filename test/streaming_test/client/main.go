package main

/*
#cgo pkg-config: libavcodec libavformat libavutil libswscale
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <stdlib.h>

// Helper to get error string
static const char* av_err_str(int errnum) {
    static char str[AV_ERROR_MAX_STRING_SIZE];
    av_strerror(errnum, str, AV_ERROR_MAX_STRING_SIZE);
    return str;
}
*/
import "C"
import (
	"bytes"
	"encoding/base64"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"log"
	"net/http"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"
	"unsafe"

	"github.com/gorilla/websocket"
)

// H264Decoder handles decoding of H.264 using libav
type H264Decoder struct {
	codecCtx    *C.AVCodecContext
	parser      *C.AVCodecParserContext
	frame       *C.AVFrame
	rgbFrame    *C.AVFrame
	packet      *C.AVPacket
	swsCtx      *C.struct_SwsContext
	frameWidth  int
	frameHeight int
	rgbBuffer   []byte
	mu          sync.Mutex
}

// NewH264Decoder creates a new libav-based H.264 decoder
func NewH264Decoder(width, height int) (*H264Decoder, error) {
	decoder := &H264Decoder{
		frameWidth:  width,
		frameHeight: height,
	}

	// Find H.264 decoder
	codec := C.avcodec_find_decoder(C.AV_CODEC_ID_H264)
	if codec == nil {
		return nil, fmt.Errorf("H.264 decoder not found")
	}

	// Allocate codec context
	decoder.codecCtx = C.avcodec_alloc_context3(codec)
	if decoder.codecCtx == nil {
		return nil, fmt.Errorf("failed to allocate codec context")
	}

	// Open codec
	if ret := C.avcodec_open2(decoder.codecCtx, codec, nil); ret < 0 {
		C.avcodec_free_context(&decoder.codecCtx)
		return nil, fmt.Errorf("failed to open codec: %s", C.GoString(C.av_err_str(ret)))
	}

	// Initialize parser for H.264 stream
	decoder.parser = C.av_parser_init(C.int(C.AV_CODEC_ID_H264))
	if decoder.parser == nil {
		C.avcodec_free_context(&decoder.codecCtx)
		return nil, fmt.Errorf("failed to initialize parser")
	}

	// Allocate frame for decoded data
	decoder.frame = C.av_frame_alloc()
	if decoder.frame == nil {
		C.av_parser_close(decoder.parser)
		C.avcodec_free_context(&decoder.codecCtx)
		return nil, fmt.Errorf("failed to allocate frame")
	}

	// Allocate frame for RGB conversion
	decoder.rgbFrame = C.av_frame_alloc()
	if decoder.rgbFrame == nil {
		C.av_frame_free(&decoder.frame)
		C.av_parser_close(decoder.parser)
		C.avcodec_free_context(&decoder.codecCtx)
		return nil, fmt.Errorf("failed to allocate RGB frame")
	}

	// Allocate packet
	decoder.packet = C.av_packet_alloc()
	if decoder.packet == nil {
		C.av_frame_free(&decoder.rgbFrame)
		C.av_frame_free(&decoder.frame)
		C.av_parser_close(decoder.parser)
		C.avcodec_free_context(&decoder.codecCtx)
		return nil, fmt.Errorf("failed to allocate packet")
	}

	// Allocate RGB buffer
	bufferSize := C.av_image_get_buffer_size(C.AV_PIX_FMT_RGB24, C.int(width), C.int(height), 1)
	decoder.rgbBuffer = make([]byte, int(bufferSize))

	log.Printf("H264Decoder initialized: %dx%d, RGB buffer size: %d", width, height, bufferSize)

	return decoder, nil
}

// initSwsContext initializes the software scaler context when we know the source format
func (d *H264Decoder) initSwsContext() error {
	if d.swsCtx != nil {
		return nil
	}

	srcWidth := d.frame.width
	srcHeight := d.frame.height
	srcFormat := d.frame.format

	d.swsCtx = C.sws_getContext(
		srcWidth, srcHeight, int32(srcFormat),
		C.int(d.frameWidth), C.int(d.frameHeight), C.AV_PIX_FMT_RGB24,
		C.SWS_BILINEAR, nil, nil, nil,
	)

	if d.swsCtx == nil {
		return fmt.Errorf("failed to create sws context")
	}

	// Setup RGB frame
	C.av_image_fill_arrays(
		&d.rgbFrame.data[0],
		&d.rgbFrame.linesize[0],
		(*C.uint8_t)(unsafe.Pointer(&d.rgbBuffer[0])),
		C.AV_PIX_FMT_RGB24,
		C.int(d.frameWidth),
		C.int(d.frameHeight),
		1,
	)

	return nil
}

// Decode decodes H.264 data and returns decoded RGB frames
func (d *H264Decoder) Decode(data []byte) ([][]byte, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	if len(data) == 0 {
		return nil, nil
	}

	var frames [][]byte
	dataPtr := (*C.uint8_t)(unsafe.Pointer(&data[0]))
	dataSize := C.int(len(data))

	for dataSize > 0 {
		// Parse the data to extract packets
		var packetData *C.uint8_t
		var packetSize C.int

		ret := C.av_parser_parse2(
			d.parser,
			d.codecCtx,
			&packetData,
			&packetSize,
			dataPtr,
			dataSize,
			C.AV_NOPTS_VALUE,
			C.AV_NOPTS_VALUE,
			0,
		)

		if ret < 0 {
			return frames, fmt.Errorf("parser error: %s", C.GoString(C.av_err_str(ret)))
		}

		dataPtr = (*C.uint8_t)(unsafe.Pointer(uintptr(unsafe.Pointer(dataPtr)) + uintptr(ret)))
		dataSize -= ret

		if packetSize > 0 {
			d.packet.data = packetData
			d.packet.size = packetSize

			// Send packet to decoder
			ret = C.avcodec_send_packet(d.codecCtx, d.packet)
			if ret < 0 {
				if ret == C.int(-C.EAGAIN) {
					continue
				}
				return frames, fmt.Errorf("send packet error: %s", C.GoString(C.av_err_str(ret)))
			}

			// Receive decoded frames
			for {
				ret = C.avcodec_receive_frame(d.codecCtx, d.frame)
				if ret == C.int(-C.EAGAIN) || ret == C.int(C.AVERROR_EOF) {
					break
				}
				if ret < 0 {
					return frames, fmt.Errorf("receive frame error: %s", C.GoString(C.av_err_str(ret)))
				}

				// Initialize sws context if needed
				if err := d.initSwsContext(); err != nil {
					return frames, err
				}

				// Convert to RGB
				C.sws_scale(
					d.swsCtx,
					&d.frame.data[0],
					&d.frame.linesize[0],
					0,
					d.frame.height,
					&d.rgbFrame.data[0],
					&d.rgbFrame.linesize[0],
				)

				// Copy RGB data
				frameCopy := make([]byte, len(d.rgbBuffer))
				copy(frameCopy, d.rgbBuffer)
				frames = append(frames, frameCopy)

				C.av_frame_unref(d.frame)
			}
		}
	}

	return frames, nil
}

// Close releases all decoder resources
func (d *H264Decoder) Close() {
	d.mu.Lock()
	defer d.mu.Unlock()

	if d.swsCtx != nil {
		C.sws_freeContext(d.swsCtx)
		d.swsCtx = nil
	}

	if d.packet != nil {
		C.av_packet_free(&d.packet)
	}

	if d.rgbFrame != nil {
		C.av_frame_free(&d.rgbFrame)
	}

	if d.frame != nil {
		C.av_frame_free(&d.frame)
	}

	if d.parser != nil {
		C.av_parser_close(d.parser)
		d.parser = nil
	}

	if d.codecCtx != nil {
		C.avcodec_free_context(&d.codecCtx)
	}
}

// FrameStats tracks frame reception statistics
type FrameStats struct {
	mu             sync.Mutex
	framesReceived int64
	bytesReceived  int64
	framesDecoded  int64
	decodeErrors   int64
	startTime      time.Time
	lastFrameTime  time.Time
}

func (s *FrameStats) RecordFrame(size int) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.framesReceived++
	s.bytesReceived += int64(size)
	s.lastFrameTime = time.Now()
}

func (s *FrameStats) RecordDecode(count int, success bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if success {
		s.framesDecoded += int64(count)
	} else {
		s.decodeErrors++
	}
}

func (s *FrameStats) Print() {
	s.mu.Lock()
	defer s.mu.Unlock()

	elapsed := time.Since(s.startTime).Seconds()
	fps := float64(s.framesReceived) / elapsed
	decodeFps := float64(s.framesDecoded) / elapsed
	bitrate := float64(s.bytesReceived*8) / elapsed / 1000 // kbps

	fmt.Printf("\r[Stats] Recv: %d | Decoded: %d | Errors: %d | Recv FPS: %.2f | Decode FPS: %.2f | Bitrate: %.2f kbps",
		s.framesReceived, s.framesDecoded, s.decodeErrors, fps, decodeFps, bitrate)
}

func (s *FrameStats) GetStats() (int64, int64, int64, float64, float64, float64) {
	s.mu.Lock()
	defer s.mu.Unlock()

	elapsed := time.Since(s.startTime).Seconds()
	fps := float64(s.framesReceived) / elapsed
	decodeFps := float64(s.framesDecoded) / elapsed
	bitrate := float64(s.bytesReceived*8) / elapsed / 1000

	return s.framesReceived, s.framesDecoded, s.decodeErrors, fps, decodeFps, bitrate
}

// WebUI manages the web interface for displaying frames
type WebUI struct {
	clients       map[*websocket.Conn]bool
	clientsMu     sync.RWMutex
	latestFrame   []byte
	frameMu       sync.RWMutex
	width         int
	height        int
	upgrader      websocket.Upgrader
	encoderConn   *websocket.Conn // Connection to encoder server
	encoderConnMu sync.Mutex
}

func NewWebUI(width, height int, encoderConn *websocket.Conn) *WebUI {
	return &WebUI{
		clients:     make(map[*websocket.Conn]bool),
		width:       width,
		height:      height,
		encoderConn: encoderConn,
		upgrader: websocket.Upgrader{
			CheckOrigin: func(r *http.Request) bool {
				return true
			},
		},
	}
}

func (w *WebUI) SetFrame(rgbData []byte) {
	// Convert RGB to JPEG for efficient transmission
	img := image.NewRGBA(image.Rect(0, 0, w.width, w.height))
	for y := 0; y < w.height; y++ {
		for x := 0; x < w.width; x++ {
			i := (y*w.width + x) * 3
			img.Set(x, y, color.RGBA{
				R: rgbData[i],
				G: rgbData[i+1],
				B: rgbData[i+2],
				A: 255,
			})
		}
	}

	var buf bytes.Buffer
	jpeg.Encode(&buf, img, &jpeg.Options{Quality: 80})

	w.frameMu.Lock()
	w.latestFrame = buf.Bytes()
	w.frameMu.Unlock()

	// Broadcast to all connected clients
	w.broadcastFrame()
}

func (w *WebUI) broadcastFrame() {
	w.frameMu.RLock()
	frame := w.latestFrame
	w.frameMu.RUnlock()

	if frame == nil {
		return
	}

	// Encode as base64 for JSON transmission
	b64Frame := base64.StdEncoding.EncodeToString(frame)
	message := []byte(fmt.Sprintf(`{"type":"frame","data":"%s"}`, b64Frame))

	w.clientsMu.RLock()
	defer w.clientsMu.RUnlock()

	for client := range w.clients {
		err := client.WriteMessage(websocket.TextMessage, message)
		if err != nil {
			client.Close()
			go func(c *websocket.Conn) {
				w.clientsMu.Lock()
				delete(w.clients, c)
				w.clientsMu.Unlock()
			}(client)
		}
	}
}

func (w *WebUI) handleWebSocket(rw http.ResponseWriter, r *http.Request) {
	conn, err := w.upgrader.Upgrade(rw, r, nil)
	if err != nil {
		log.Printf("WebSocket upgrade error: %v", err)
		return
	}

	w.clientsMu.Lock()
	w.clients[conn] = true
	w.clientsMu.Unlock()

	log.Printf("Web client connected. Total: %d", len(w.clients))

	// Handle messages from web client (control commands)
	for {
		messageType, message, err := conn.ReadMessage()
		if err != nil {
			break
		}

		// Forward text messages to encoder server as control commands
		if messageType == websocket.TextMessage {
			w.encoderConnMu.Lock()
			if w.encoderConn != nil {
				err := w.encoderConn.WriteMessage(websocket.TextMessage, message)
				if err != nil {
					log.Printf("Error forwarding control message: %v", err)
				} else {
					log.Printf("Forwarded control message: %s", string(message))
				}
			}
			w.encoderConnMu.Unlock()
		}
	}

	w.clientsMu.Lock()
	delete(w.clients, conn)
	w.clientsMu.Unlock()
	conn.Close()

	log.Printf("Web client disconnected. Total: %d", len(w.clients))
}

func (w *WebUI) handleIndex(rw http.ResponseWriter, r *http.Request) {
	html := fmt.Sprintf(`<!DOCTYPE html>
<html>
<head>
    <title>Video Stream Viewer</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #1a1a2e;
            color: #eee;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
        }
        h1 {
            margin-bottom: 20px;
            color: #4ecca3;
        }
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 20px;
        }
        #videoCanvas {
            border: 3px solid #4ecca3;
            border-radius: 8px;
            background: #000;
        }
        .stats {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            padding: 15px;
            background: #16213e;
            border-radius: 8px;
            min-width: 500px;
        }
        .stat-item {
            text-align: center;
            padding: 10px;
            background: #1a1a2e;
            border-radius: 6px;
        }
        .stat-label {
            font-size: 12px;
            color: #888;
            margin-bottom: 5px;
        }
        .stat-value {
            font-size: 18px;
            font-weight: bold;
            color: #4ecca3;
        }
        .status {
            padding: 8px 16px;
            border-radius: 20px;
            font-size: 14px;
        }
        .status.connected {
            background: #2d6a4f;
            color: #95d5b2;
        }
        .status.disconnected {
            background: #6a2d2d;
            color: #d59595;
        }
        .controls {
            display: flex;
            flex-direction: column;
            gap: 15px;
            padding: 20px;
            background: #16213e;
            border-radius: 8px;
            min-width: 500px;
        }
        .controls h2 {
            color: #4ecca3;
            font-size: 16px;
            margin-bottom: 5px;
        }
        .control-group {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }
        .control-row {
            display: flex;
            align-items: center;
            gap: 15px;
        }
        .control-row label {
            min-width: 100px;
            font-size: 14px;
            color: #aaa;
        }
        .control-row input[type="range"] {
            flex: 1;
            accent-color: #4ecca3;
        }
        .control-row .value {
            min-width: 60px;
            text-align: right;
            font-weight: bold;
            color: #4ecca3;
        }
        .quality-presets {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        .preset-btn {
            padding: 8px 16px;
            border: none;
            border-radius: 6px;
            background: #1a1a2e;
            color: #eee;
            cursor: pointer;
            transition: all 0.2s;
            font-size: 13px;
        }
        .preset-btn:hover {
            background: #4ecca3;
            color: #1a1a2e;
        }
        .preset-btn.active {
            background: #4ecca3;
            color: #1a1a2e;
        }
        .action-btn {
            padding: 10px 20px;
            border: none;
            border-radius: 6px;
            background: #4ecca3;
            color: #1a1a2e;
            cursor: pointer;
            font-weight: bold;
            transition: all 0.2s;
        }
        .action-btn:hover {
            background: #3db892;
        }
    </style>
</head>
<body>
    <h1>🎬 Video Stream Viewer</h1>
    <div class="container">
        <div id="statusContainer">
            <span id="status" class="status disconnected">Disconnected</span>
        </div>
        <canvas id="videoCanvas" width="%d" height="%d"></canvas>
        
        <div class="controls">
            <h2>🎛️ Quality Controls</h2>
            
            <div class="control-group">
                <label>Quality Presets:</label>
                <div class="quality-presets">
                    <button class="preset-btn" onclick="setQuality('low')">Low (CRF 35)</button>
                    <button class="preset-btn" onclick="setQuality('medium')">Medium (CRF 28)</button>
                    <button class="preset-btn active" onclick="setQuality('high')">High (CRF 23)</button>
                    <button class="preset-btn" onclick="setQuality('ultra')">Ultra (CRF 18)</button>
                </div>
            </div>
            
            <div class="control-group">
                <div class="control-row">
                    <label>CRF (Quality):</label>
                    <input type="range" id="crfSlider" min="0" max="51" value="23" oninput="updateCrfDisplay()">
                    <span class="value" id="crfValue">23</span>
                </div>
                <div style="font-size: 11px; color: #666; margin-left: 115px;">
                    0 = Lossless, 51 = Worst quality. Recommended: 18-28
                </div>
            </div>
            
            <div class="control-group">
                <div class="control-row">
                    <label>GOP Size:</label>
                    <input type="range" id="gopSlider" min="1" max="300" value="60" oninput="updateGopDisplay()">
                    <span class="value" id="gopValue">60</span>
                </div>
                <div style="font-size: 11px; color: #666; margin-left: 115px;">
                    Keyframe interval (lower = more keyframes, higher bandwidth)
                </div>
            </div>
            
            <div class="control-row" style="margin-top: 10px;">
                <button class="action-btn" onclick="applySettings()">Apply Settings</button>
                <button class="action-btn" onclick="forceKeyframe()" style="background: #e76f51;">Force Keyframe</button>
            </div>
        </div>
        
        <div class="stats">
            <div class="stat-item">
                <div class="stat-label">Frames Received</div>
                <div class="stat-value" id="framesReceived">0</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">FPS</div>
                <div class="stat-value" id="fps">0.00</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">Latency</div>
                <div class="stat-value" id="latency">0 ms</div>
            </div>
        </div>
    </div>

    <script>
        const canvas = document.getElementById('videoCanvas');
        const ctx = canvas.getContext('2d');
        const statusEl = document.getElementById('status');
        const framesReceivedEl = document.getElementById('framesReceived');
        const fpsEl = document.getElementById('fps');
        const latencyEl = document.getElementById('latency');

        let ws = null;
        let frameCount = 0;
        let lastFrameTime = Date.now();
        let fpsCounter = 0;
        let lastFpsUpdate = Date.now();

        function updateCrfDisplay() {
            document.getElementById('crfValue').textContent = document.getElementById('crfSlider').value;
        }

        function updateGopDisplay() {
            document.getElementById('gopValue').textContent = document.getElementById('gopSlider').value;
        }

        function setQuality(preset) {
            const presets = {
                'low': { crf: 35, gop: 120 },
                'medium': { crf: 28, gop: 90 },
                'high': { crf: 23, gop: 60 },
                'ultra': { crf: 18, gop: 30 }
            };
            
            const p = presets[preset];
            document.getElementById('crfSlider').value = p.crf;
            document.getElementById('gopSlider').value = p.gop;
            updateCrfDisplay();
            updateGopDisplay();
            
            // Update active button
            document.querySelectorAll('.preset-btn').forEach(btn => btn.classList.remove('active'));
            event.target.classList.add('active');
            
            // Apply immediately
            applySettings();
        }

        function applySettings() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const crf = document.getElementById('crfSlider').value;
                const gop = document.getElementById('gopSlider').value;
                const msg = 'crf=' + crf + '&gop_size=' + gop;
                ws.send(msg);
                console.log('Sent settings:', msg);
            }
        }

        function forceKeyframe() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                ws.send('keyframe=1');
                console.log('Requested keyframe');
            }
        }

        function connect() {
            ws = new WebSocket('ws://' + window.location.host + '/ws');

            ws.onopen = function() {
                statusEl.textContent = 'Connected';
                statusEl.className = 'status connected';
                console.log('Connected to server');
            };

            ws.onclose = function() {
                statusEl.textContent = 'Disconnected';
                statusEl.className = 'status disconnected';
                console.log('Disconnected from server');
                // Reconnect after 2 seconds
                setTimeout(connect, 2000);
            };

            ws.onerror = function(err) {
                console.error('WebSocket error:', err);
            };

            ws.onmessage = function(event) {
                const now = Date.now();
                const data = JSON.parse(event.data);

                if (data.type === 'frame') {
                    const img = new Image();
                    img.onload = function() {
                        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
                    };
                    img.src = 'data:image/jpeg;base64,' + data.data;

                    frameCount++;
                    fpsCounter++;
                    framesReceivedEl.textContent = frameCount;

                    // Update latency
                    const latency = now - lastFrameTime;
                    lastFrameTime = now;
                    latencyEl.textContent = latency + ' ms';

                    // Update FPS every second
                    if (now - lastFpsUpdate >= 1000) {
                        fpsEl.textContent = fpsCounter.toFixed(2);
                        fpsCounter = 0;
                        lastFpsUpdate = now;
                    }
                }
            };
        }

        connect();
    </script>
</body>
</html>`, w.width, w.height)

	rw.Header().Set("Content-Type", "text/html")
	rw.Write([]byte(html))
}

func main() {
	// Command line flags
	serverAddr := flag.String("server", "localhost:8765", "WebSocket server address (encoder)")
	width := flag.Int("width", 640, "Video frame width")
	height := flag.Int("height", 480, "Video frame height")
	webPort := flag.Int("web-port", 8080, "HTTP port for web viewer")
	verbose := flag.Bool("verbose", false, "Enable verbose logging")
	flag.Parse()

	log.Printf("Connecting to ws://%s", *serverAddr)

	// Setup signal handling
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	// Connect to WebSocket server
	wsURL := fmt.Sprintf("ws://%s", *serverAddr)
	conn, _, err := websocket.DefaultDialer.Dial(wsURL, nil)
	if err != nil {
		log.Fatalf("Failed to connect to server: %v", err)
	}
	defer conn.Close()

	log.Println("Connected to encoder server")

	// Create decoder
	decoder, err := NewH264Decoder(*width, *height)
	if err != nil {
		log.Fatalf("Failed to create decoder: %v", err)
	}
	defer decoder.Close()

	log.Println("Decoder initialized (using libav)")

	// Create Web UI with encoder connection for control commands
	webUI := NewWebUI(*width, *height, conn)

	// Setup HTTP server for web viewer
	http.HandleFunc("/", webUI.handleIndex)
	http.HandleFunc("/ws", webUI.handleWebSocket)

	go func() {
		addr := fmt.Sprintf(":%d", *webPort)
		log.Printf("Web viewer available at http://localhost%s", addr)
		if err := http.ListenAndServe(addr, nil); err != nil {
			log.Fatalf("HTTP server error: %v", err)
		}
	}()

	// Initialize stats
	stats := &FrameStats{
		startTime: time.Now(),
	}

	// Channel for received frames
	frameChan := make(chan []byte, 100)
	done := make(chan struct{})

	// Frame receiver goroutine
	go func() {
		defer close(frameChan)
		for {
			select {
			case <-done:
				return
			default:
				messageType, data, err := conn.ReadMessage()
				if err != nil {
					if websocket.IsCloseError(err, websocket.CloseNormalClosure) {
						log.Println("Server closed connection")
					} else {
						log.Printf("Read error: %v", err)
					}
					return
				}

				if messageType == websocket.BinaryMessage {
					stats.RecordFrame(len(data))

					// Copy data for the channel
					frameCopy := make([]byte, len(data))
					copy(frameCopy, data)

					select {
					case frameChan <- frameCopy:
					default:
						if *verbose {
							log.Println("Frame dropped - channel full")
						}
					}
				}
			}
		}
	}()

	// Frame decoder goroutine
	go func() {
		for encodedData := range frameChan {
			// Decode frame(s)
			decodedFrames, err := decoder.Decode(encodedData)
			if err != nil {
				stats.RecordDecode(0, false)
				if *verbose {
					log.Printf("Decode error: %v", err)
				}
				continue
			}

			if len(decodedFrames) > 0 {
				stats.RecordDecode(len(decodedFrames), true)

				// Send the latest frame to web UI
				webUI.SetFrame(decodedFrames[len(decodedFrames)-1])
			}
		}
	}()

	// Stats printer goroutine
	statsTicker := time.NewTicker(1 * time.Second)
	defer statsTicker.Stop()

	go func() {
		for {
			select {
			case <-statsTicker.C:
				stats.Print()
			case <-done:
				return
			}
		}
	}()

	// Wait for signal
	<-sigChan
	fmt.Println("\nShutting down...")

	close(done)
	conn.WriteMessage(websocket.CloseMessage,
		websocket.FormatCloseMessage(websocket.CloseNormalClosure, ""))

	// Final stats
	fmt.Println()
	stats.Print()
	fmt.Println()

	log.Println("Client shutdown complete")
}
