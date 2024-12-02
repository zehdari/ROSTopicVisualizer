import {
  __commonJS
} from "./chunk-V4OQ3NZ2.js";

// node_modules/@xterm/addon-fit/lib/addon-fit.js
var require_addon_fit = __commonJS({
  "node_modules/@xterm/addon-fit/lib/addon-fit.js"(exports, module) {
    !function(e, t) {
      "object" == typeof exports && "object" == typeof module ? module.exports = t() : "function" == typeof define && define.amd ? define([], t) : "object" == typeof exports ? exports.FitAddon = t() : e.FitAddon = t();
    }(self, () => (() => {
      "use strict";
      var e = {};
      return (() => {
        var t = e;
        Object.defineProperty(t, "__esModule", { value: true }), t.FitAddon = void 0, t.FitAddon = class {
          activate(e2) {
            this._terminal = e2;
          }
          dispose() {
          }
          fit() {
            const e2 = this.proposeDimensions();
            if (!e2 || !this._terminal || isNaN(e2.cols) || isNaN(e2.rows)) return;
            const t2 = this._terminal._core;
            this._terminal.rows === e2.rows && this._terminal.cols === e2.cols || (t2._renderService.clear(), this._terminal.resize(e2.cols, e2.rows));
          }
          proposeDimensions() {
            if (!this._terminal) return;
            if (!this._terminal.element || !this._terminal.element.parentElement) return;
            const e2 = this._terminal._core, t2 = e2._renderService.dimensions;
            if (0 === t2.css.cell.width || 0 === t2.css.cell.height) return;
            const r = 0 === this._terminal.options.scrollback ? 0 : e2.viewport.scrollBarWidth, i = window.getComputedStyle(this._terminal.element.parentElement), o = parseInt(i.getPropertyValue("height")), s = Math.max(0, parseInt(i.getPropertyValue("width"))), n = window.getComputedStyle(this._terminal.element), l = o - (parseInt(n.getPropertyValue("padding-top")) + parseInt(n.getPropertyValue("padding-bottom"))), a = s - (parseInt(n.getPropertyValue("padding-right")) + parseInt(n.getPropertyValue("padding-left"))) - r;
            return { cols: Math.max(2, Math.floor(a / t2.css.cell.width)), rows: Math.max(1, Math.floor(l / t2.css.cell.height)) };
          }
        };
      })(), e;
    })());
  }
});
export default require_addon_fit();
//# sourceMappingURL=@xterm_addon-fit.js.map
