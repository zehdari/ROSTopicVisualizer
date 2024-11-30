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
    });

    const fitAddon = new FitAddon();
    term.loadAddon(fitAddon);
    term.loadAddon(new WebLinksAddon());

    xtermRef.current = term;
    fitAddonRef.current = fitAddon;

    term.open(terminalRef.current);

    fitAddon.fit();

    const socket = io(NETWORK_CONFIG.FLASK_SERVER_URL);
    socketRef.current = socket;

    socket.on("connect", () => {
      term.writeln("Connected to terminal server");
    });

    socket.on("disconnect", () => {
      term.writeln("Disconnected from terminal server");
    });

    socket.on("terminal_output", (data) => {
      term.write(data.output);
    });

    term.onData((data) => {
      socket.emit("terminal_input", { input: data });
    });

    term.onResize(({ rows, cols }) => {
      socket.emit("resize", { rows, cols });
    });

    const handleResize = () => {
      fitAddon.fit();
      const { rows, cols } = term;
      socket.emit("resize", { rows, cols });
    };

    window.addEventListener("resize", handleResize);

    fitAddon.fit();
    const { rows, cols } = term;
    socket.emit("resize", { rows, cols });

    return () => {
      window.removeEventListener("resize", handleResize);
      socket.disconnect();
      term.dispose();
    };
  }, []);

  return <div ref={terminalRef} className="terminal-container" />;
};

export default TerminalComponent;
