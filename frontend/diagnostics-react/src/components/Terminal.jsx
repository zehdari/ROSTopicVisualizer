import React, { useEffect, useRef } from "react";
import { Terminal } from "@xterm/xterm";
import { FitAddon } from "@xterm/addon-fit";
import { WebLinksAddon } from "@xterm/addon-web-links";
import { io } from "socket.io-client";
import "@xterm/xterm/css/xterm.css";
import { NETWORK_CONFIG } from "../config/networkConfig";
import "../styles/Terminal.css";

const TerminalComponent = () => {
  const terminalRef = useRef(null);
  const xtermRef = useRef(null);
  const socketRef = useRef(null);
  const fitAddonRef = useRef(null);

  useEffect(() => {
    if (!terminalRef.current) {
      console.warn("Terminal container ref not ready");
      return;
    }

    try {
      const term = new Terminal({
        cursorBlink: true,
        cursorStyle: "block",
        fontSize: 14,
        fontFamily: 'Menlo, Monaco, "Courier New", monospace',
        theme: {
          background: "#1e1e1e",
          foreground: "#ffffff",
        },
        borderWidth: 0,
        windowsMode: false,
        rendererType: "canvas",
        allowTransparency: false,
        scrollback: 1000,
        cols: 80,
        rows: 24,
        convertEol: true,
        rightClickSelectsWord: true,
      });

      const fitAddon = new FitAddon();
      term.loadAddon(fitAddon);
      term.loadAddon(new WebLinksAddon());

      xtermRef.current = term;
      fitAddonRef.current = fitAddon;

      while (terminalRef.current.firstChild) {
        terminalRef.current.removeChild(terminalRef.current.firstChild);
      }

      term.open(terminalRef.current);

      // Ensure proper fitting after the terminal is mounted
      requestAnimationFrame(() => {
        try {
          fitAddon.fit();
          term.refresh(0, term.rows - 1);
        } catch (error) {
          console.error("Error during initial fit:", error);
        }
      });

      const socket = io(NETWORK_CONFIG.FLASK_SERVER_URL, {
        reconnection: true,
        reconnectionAttempts: 5,
        reconnectionDelay: 1000,
      });

      socketRef.current = socket;

      socket.on("connect", () => {
        term.writeln("Connected to terminal server");
      });

      socket.on("connect_error", (error) => {
        term.writeln(`Connection error: ${error.message}`);
      });

      socket.on("disconnect", () => {
        term.writeln("Disconnected from terminal server");
      });

      socket.on("terminal_output", (data) => {
        if (data && data.output) {
          term.write(data.output);
        }
      });

      term.onData((data) => {
        if (socket.connected) {
          socket.emit("terminal_input", { input: data });
        }
      });

      let resizeTimeout;
      const handleResize = () => {
        clearTimeout(resizeTimeout);
        resizeTimeout = setTimeout(() => {
          try {
            if (fitAddonRef.current && term.element) {
              fitAddonRef.current.fit();
              const { rows, cols } = term;
              if (socket.connected) {
                socket.emit("resize", { rows, cols });
              }
              term.refresh(0, term.rows - 1);
            }
          } catch (error) {
            console.error("Error handling resize:", error);
          }
        }, 100);
      };

      window.addEventListener("resize", handleResize);

      return () => {
        clearTimeout(resizeTimeout);
        window.removeEventListener("resize", handleResize);
        if (socket.connected) {
          socket.disconnect();
        }
        term.dispose();
      };
    } catch (error) {
      console.error("Error initializing terminal:", error);
    }
  }, []);

  return (
    <div
      ref={terminalRef}
      className="terminal-container"
      style={{
        width: "100%",
        height: "100%",
        minHeight: "300px",
        overflow: "hidden",
        position: "relative",
      }}
    />
  );
};

export default TerminalComponent;
