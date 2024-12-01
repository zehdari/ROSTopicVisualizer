import React, { useEffect, useRef, useState } from "react";
import REGL from "regl";
import { mat4, vec3 } from "gl-matrix";
import { useRosTopic } from "../utils/useRosTopic";

const vertexShader = `
  precision mediump float;
  attribute vec3 position;
  attribute vec3 color;
  uniform mat4 view;
  uniform mat4 projection;
  varying vec3 fragColor;
  void main() {
    fragColor = color;
    gl_Position = projection * view * vec4(position, 1);
    gl_PointSize = 3.0;
  }
`;

const fragmentShader = `
  precision mediump float;
  varying vec3 fragColor;
  void main() {
    gl_FragColor = vec4(fragColor, 1.0);
  }
`;

function base64ToArrayBuffer(base64) {
  const binaryString = atob(base64);
  const bytes = new Uint8Array(binaryString.length);
  for (let i = 0; i < binaryString.length; i++) {
    bytes[i] = binaryString.charCodeAt(i);
  }
  return bytes.buffer;
}

const parsePointCloud2 = (message) => {
  try {
    const positions = [];
    const colors = [];

    // Convert base64 data to ArrayBuffer
    const buffer = base64ToArrayBuffer(message.data);
    const dataView = new DataView(buffer);

    // Find field offsets
    const fieldOffsets = {};
    message.fields.forEach((field) => {
      fieldOffsets[field.name] = field.offset;
    });

    const stride = message.point_step;
    const pointCount = message.width * message.height;

    // Track bounds for color normalization
    let minY = Infinity;
    let maxY = -Infinity;

    // First pass to find Y bounds
    for (let i = 0; i < pointCount; i++) {
      const baseOffset = i * stride;
      const y = dataView.getFloat32(baseOffset + fieldOffsets.y, true);
      if (!isNaN(y)) {
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
      }
    }

    // Second pass to read points and generate colors
    for (let i = 0; i < pointCount; i++) {
      const baseOffset = i * stride;
      const x = dataView.getFloat32(baseOffset + fieldOffsets.x, true);
      const y = dataView.getFloat32(baseOffset + fieldOffsets.y, true);
      const z = dataView.getFloat32(baseOffset + fieldOffsets.z, true);

      // Skip invalid points
      if (isNaN(x) || isNaN(y) || isNaN(z)) continue;

      positions.push(x, y, z);

      // Normalize height to 0-1 range and create a color gradient
      const heightNorm = (y - minY) / (maxY - minY);

      // Simple blue to red gradient
      colors.push(
        heightNorm, // R increases with height
        0.2, // G constant
        1.0 - heightNorm // B decreases with height
      );
    }

    return {
      positions: new Float32Array(positions),
      colors: new Float32Array(colors),
      count: positions.length / 3,
    };
  } catch (error) {
    console.error("Error parsing PointCloud2:", error);
    return null;
  }
};

const DEFAULT_CAMERA = {
  radius: 3.464,
  theta: Math.PI / 4,
  phi: Math.PI / 4,
  eye: [2, 2, 2],
  center: [0, 0, 0],
  up: [0, 1, 0],
  position: [0, 0, 0],
  moveSpeed: 0.1,
  rotateSpeed: 0.03,
};

const PointCloudViewer = ({
  topicName = "/pointcloud_topic",
  frameId = "base_link",
  onReset,
}) => {
  const canvasRef = useRef(null);
  const reglRef = useRef(null);
  const animationFrameRef = useRef();
  const keysPressed = useRef({});
  const [isFocused, setIsFocused] = useState(false);

  // Modified camera reference to include rotation speed
  const cameraRef = useRef({ ...DEFAULT_CAMERA });

  const pointCloudMessage = useRosTopic(
    topicName,
    "sensor_msgs/msg/PointCloud2"
  );
  const drawCommandRef = useRef(null);
  const isDraggingRef = useRef(false);
  const lastMouseRef = useRef({ x: 0, y: 0 });

  const touchRef = useRef({
    isRotating: false,
    isPanning: false,
    lastX: 0,
    lastY: 0,
    lastDistance: 0,
    lastCenter: { x: 0, y: 0 },
  });

  const resetCamera = () => {
    // Reset camera to default position and orientation
    cameraRef.current = {
      ...DEFAULT_CAMERA,
      position: [...DEFAULT_CAMERA.position],
      theta: DEFAULT_CAMERA.theta,
      phi: DEFAULT_CAMERA.phi,
      eye: [...DEFAULT_CAMERA.eye],
      up: [...DEFAULT_CAMERA.up],
    };

    // Reset touch states
    touchRef.current = {
      isRotating: false,
      isPanning: false,
      lastX: 0,
      lastY: 0,
      lastDistance: 0,
      lastCenter: { x: 0, y: 0 },
    };

    // Reset mouse states
    isDraggingRef.current = false;
    lastMouseRef.current = { x: 0, y: 0 };

    // Reset keyboard states
    keysPressed.current = {};

    // Force a camera position update
    updateCameraPosition();

    // Force a re-render if needed
    if (reglRef.current && drawCommandRef.current) {
      const canvas = canvasRef.current;
      if (canvas) {
        const projection = mat4.perspective(
          mat4.create(),
          Math.PI / 4,
          canvas.width / canvas.height,
          0.1,
          100.0
        );

        const view = mat4.lookAt(
          mat4.create(),
          cameraRef.current.eye,
          cameraRef.current.center,
          cameraRef.current.up
        );

        reglRef.current.clear({
          color: [0.1, 0.1, 0.1, 1],
          depth: 1,
        });

        drawCommandRef.current({
          positions: pointData?.positions,
          colors: pointData?.colors,
          count: pointData?.count,
          view,
          projection,
        });
      }
    }
  };
  // Pass the resetCamera function to the onReset callback
  useEffect(() => {
    if (onReset) {
      onReset(resetCamera);
    }
  }, [onReset]);

  // Add keyboard controls
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (!isFocused) return;

      keysPressed.current[e.key.toLowerCase()] = true;

      if (e.key.toLowerCase() === "r") {
        resetCamera();
      }
    };

    const handleKeyUp = (e) => {
      if (!isFocused) return;

      keysPressed.current[e.key.toLowerCase()] = false;
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [isFocused]);

  useEffect(() => {
    if (!isFocused) return;
    const preventSpaceScroll = (e) => {
      if (e.key === " " || e.code === "Space") {
        e.preventDefault();
      }
    };

    window.addEventListener("keydown", preventSpaceScroll);
    return () => window.removeEventListener("keydown", preventSpaceScroll);
  }, [isFocused]);

  // Update camera position based on WASD, Space/Shift, and Q/E keys
  const updateCameraPosition = () => {
    const camera = cameraRef.current;
    const moveSpeed = camera.moveSpeed;

    // Calculate forward and right vectors based on camera orientation
    const forward = vec3.fromValues(
      Math.sin(camera.theta) * Math.sin(camera.phi),
      0,
      Math.cos(camera.theta) * Math.sin(camera.phi)
    );
    const right = vec3.fromValues(
      Math.cos(camera.theta),
      0,
      -Math.sin(camera.theta)
    );

    // Normalize vectors
    vec3.normalize(forward, forward);
    vec3.normalize(right, right);

    // Update position based on keys pressed
    if (keysPressed.current["w"]) {
      camera.position[0] -= forward[0] * moveSpeed;
      camera.position[2] -= forward[2] * moveSpeed;
    }
    if (keysPressed.current["s"]) {
      camera.position[0] += forward[0] * moveSpeed;
      camera.position[2] += forward[2] * moveSpeed;
    }
    if (keysPressed.current["a"]) {
      camera.position[0] -= right[0] * moveSpeed;
      camera.position[2] -= right[2] * moveSpeed;
    }
    if (keysPressed.current["d"]) {
      camera.position[0] += right[0] * moveSpeed;
      camera.position[2] += right[2] * moveSpeed;
    }

    // Vertical movement with Space and Shift
    if (keysPressed.current[" "]) {
      camera.position[1] += moveSpeed;
    }
    if (keysPressed.current["shift"]) {
      camera.position[1] -= moveSpeed;
    }

    // Rotation with Q and E (around base_link)
    if (keysPressed.current["q"] || keysPressed.current["e"]) {
      const rotationDelta =
        camera.rotateSpeed * (keysPressed.current["q"] ? 1 : -1);
      camera.theta += rotationDelta;
    }

    // Calculate eye position relative to base_link (0,0,0), not current position
    const distance = Math.sqrt(
      camera.position[0] * camera.position[0] +
        camera.position[2] * camera.position[2]
    );

    camera.eye = [
      camera.position[0] +
        camera.radius * Math.sin(camera.theta) * Math.sin(camera.phi),
      camera.position[1] + camera.radius * Math.cos(camera.phi),
      camera.position[2] +
        camera.radius * Math.cos(camera.theta) * Math.sin(camera.phi),
    ];

    camera.center = [...camera.position];
  };

  // Previous initialization effects remain the same
  useEffect(() => {
    const canvas = canvasRef.current;
    try {
      const regl = REGL({
        canvas,
        extensions: ["angle_instanced_arrays"],
        attributes: { antialias: true },
      });
      reglRef.current = regl;

      drawCommandRef.current = regl({
        vert: vertexShader,
        frag: fragmentShader,
        attributes: {
          position: regl.prop("positions"),
          color: regl.prop("colors"),
        },
        uniforms: {
          view: regl.prop("view"),
          projection: regl.prop("projection"),
        },
        count: regl.prop("count"),
        primitive: "points",
      });
    } catch (error) {
      console.error("WebGL initialization failed:", error);
    }

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      if (reglRef.current) {
        reglRef.current.destroy();
      }
    };
  }, []);

  // Previous mouse control effect remains largely the same
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const handleMouseDown = (e) => {
      isDraggingRef.current = true;
      lastMouseRef.current = { x: e.clientX, y: e.clientY };
    };

    const handleMouseUp = () => {
      isDraggingRef.current = false;
    };

    const handleMouseLeave = () => {
      isDraggingRef.current = false;
    };

    const handleMouseMove = (e) => {
      if (!isDraggingRef.current) return;

      const dx = e.clientX - lastMouseRef.current.x;
      const dy = e.clientY - lastMouseRef.current.y;
      const camera = cameraRef.current;
      const speed = 0.005;

      camera.theta -= dx * speed;
      camera.phi = Math.max(
        0.1,
        Math.min(Math.PI - 0.1, camera.phi + dy * speed)
      );

      updateCameraPosition(); // Update camera position after rotation

      lastMouseRef.current = { x: e.clientX, y: e.clientY };
    };

    const handleWheel = (e) => {
      e.preventDefault();
      const camera = cameraRef.current;
      const zoomSpeed = 0.001;
      camera.radius = Math.max(0.1, camera.radius * (1 - e.deltaY * zoomSpeed));

      updateCameraPosition(); // Update camera position after zoom
    };

    canvas.addEventListener("mousedown", handleMouseDown);
    canvas.addEventListener("mouseup", handleMouseUp);
    canvas.addEventListener("mouseleave", handleMouseLeave);
    canvas.addEventListener("mousemove", handleMouseMove);
    canvas.addEventListener("wheel", handleWheel, { passive: false });
    window.addEventListener("mouseup", handleMouseUp);
    window.addEventListener("blur", handleMouseUp);

    return () => {
      canvas.removeEventListener("mousedown", handleMouseDown);
      canvas.removeEventListener("mouseup", handleMouseUp);
      canvas.removeEventListener("mouseleave", handleMouseLeave);
      canvas.removeEventListener("mousemove", handleMouseMove);
      canvas.removeEventListener("wheel", handleWheel);
      window.removeEventListener("mouseup", handleMouseUp);
      window.removeEventListener("blur", handleMouseUp);
    };
  }, []);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const handleTouchStart = (e) => {
      e.preventDefault();
      const touches = e.touches;

      if (touches.length === 1) {
        // Single touch for rotation
        touchRef.current.isRotating = true;
        touchRef.current.lastX = touches[0].clientX;
        touchRef.current.lastY = touches[0].clientY;
      } else if (touches.length === 2) {
        // Two touches for pinch zoom and pan
        touchRef.current.isPanning = true;
        const dx = touches[0].clientX - touches[1].clientX;
        const dy = touches[0].clientY - touches[1].clientY;
        touchRef.current.lastDistance = Math.sqrt(dx * dx + dy * dy);
        touchRef.current.lastX = (touches[0].clientX + touches[1].clientX) / 2;
        touchRef.current.lastY = (touches[0].clientY + touches[1].clientY) / 2;
      }
    };

    const handleTouchMove = (e) => {
      e.preventDefault();
      const touches = e.touches;
      const camera = cameraRef.current;

      if (touches.length === 1 && touchRef.current.isRotating) {
        // Handle rotation
        const dx = touches[0].clientX - touchRef.current.lastX;
        const dy = touches[0].clientY - touchRef.current.lastY;
        const speed = 0.005;

        camera.theta -= dx * speed;
        camera.phi = Math.max(
          0.1,
          Math.min(Math.PI - 0.1, camera.phi + dy * speed)
        );

        touchRef.current.lastX = touches[0].clientX;
        touchRef.current.lastY = touches[0].clientY;
      } else if (touches.length === 2) {
        // Handle zoom and pan
        const dx = touches[0].clientX - touches[1].clientX;
        const dy = touches[0].clientY - touches[1].clientY;
        const distance = Math.sqrt(dx * dx + dy * dy);

        // Zoom
        const deltaZoom = (distance - touchRef.current.lastDistance) * 0.01;
        camera.radius = Math.max(0.1, camera.radius * (1 - deltaZoom));

        // Pan
        const centerX = (touches[0].clientX + touches[1].clientX) / 2;
        const centerY = (touches[0].clientY + touches[1].clientY) / 2;
        const panX = centerX - touchRef.current.lastX;
        const panY = centerY - touchRef.current.lastY;
        const panSpeed = 0.005;

        camera.position[0] += panX * panSpeed;
        camera.position[1] -= panY * panSpeed;

        touchRef.current.lastDistance = distance;
        touchRef.current.lastX = centerX;
        touchRef.current.lastY = centerY;
      }

      updateCameraPosition();
    };

    const handleTouchEnd = (e) => {
      e.preventDefault();
      touchRef.current.isRotating = false;
      touchRef.current.isPanning = false;
    };

    // Add touch event listeners
    canvas.addEventListener("touchstart", handleTouchStart, { passive: false });
    canvas.addEventListener("touchmove", handleTouchMove, { passive: false });
    canvas.addEventListener("touchend", handleTouchEnd, { passive: false });
    canvas.addEventListener("touchcancel", handleTouchEnd, { passive: false });

    return () => {
      canvas.removeEventListener("touchstart", handleTouchStart);
      canvas.removeEventListener("touchmove", handleTouchMove);
      canvas.removeEventListener("touchend", handleTouchEnd);
      canvas.removeEventListener("touchcancel", handleTouchEnd);
    };
  }, []);

  // Modified rendering effect to include camera position updates
  useEffect(() => {
    if (!pointCloudMessage || !reglRef.current || !drawCommandRef.current)
      return;

    const pointData = parsePointCloud2(pointCloudMessage);
    if (!pointData) return;

    const projection = mat4.perspective(
      mat4.create(),
      Math.PI / 4,
      canvasRef.current.width / canvasRef.current.height,
      0.1,
      100.0
    );

    const render = () => {
      updateCameraPosition(); // Update camera position each frame

      const view = mat4.lookAt(
        mat4.create(),
        cameraRef.current.eye,
        cameraRef.current.center,
        cameraRef.current.up
      );

      reglRef.current.clear({
        color: [0.1, 0.1, 0.1, 1],
        depth: 1,
      });

      drawCommandRef.current({
        positions: pointData.positions,
        colors: pointData.colors,
        count: pointData.count,
        view,
        projection,
      });

      animationFrameRef.current = requestAnimationFrame(render);
    };

    render();

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
    };
  }, [pointCloudMessage]);

  return (
    <div className="w-full h-96 border-2 border-gray-500 rounded-lg relative bg-gray-800">
      <canvas
        ref={canvasRef}
        className="w-full h-full"
        width={350}
        height={300}
        style={{
          display: "block",
          touchAction: "none", // Prevent default touch behaviors
        }}
        tabIndex={0}
        onFocus={() => setIsFocused(true)}
        onBlur={() => setIsFocused(false)}
      />
    </div>
  );
};

export default PointCloudViewer;
