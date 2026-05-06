/* ============================================================
   adi.js — Attitude Director Indicator (Artificial Horizon)
   Tufte: flat colors, no gradients.
   Victor: target pitch/roll ghost overlay.
   ============================================================ */

'use strict';

window.ADI = (function () {

    let canvas, ctx;
    let w, h;
    const DEG = Math.PI / 180;
    const RAD = 180 / Math.PI;

    const GHOST_DASH = [6, 4];

    // Dynamic theme colors
    function tc() { return window._themeColors || Theme.CANVAS.light; }

    // Pixels per degree of pitch
    const PPD = 4.5;

    function init(canvasEl) {
        canvas = canvasEl;
        ctx = canvas.getContext('2d');
        resize();
        window.addEventListener('resize', resize);
    }

    function resize() {
        const rect = canvas.parentElement.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;
        w = rect.width;
        h = rect.height;
        canvas.width = w * dpr;
        canvas.height = h * dpr;
        canvas.style.width = w + 'px';
        canvas.style.height = h + 'px';
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    function draw(roll, pitch, targetRoll, targetPitch) {
        if (!ctx) return;
        const c = tc();

        const cx = w / 2;
        const cy = h / 2;
        const rollDeg = roll * RAD;
        const pitchDeg = pitch * RAD;
        const pitchPx = pitchDeg * PPD;

        ctx.clearRect(0, 0, w, h);
        ctx.save();

        ctx.translate(cx, cy);
        ctx.rotate(-roll);

        const bigR = Math.max(w, h) * 2;
        ctx.fillStyle = c.sky;
        ctx.fillRect(-bigR, -bigR + pitchPx, bigR * 2, bigR);
        ctx.fillStyle = c.ground;
        ctx.fillRect(-bigR, pitchPx, bigR * 2, bigR);
        ctx.strokeStyle = c.horizon;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(-bigR, pitchPx);
        ctx.lineTo(bigR, pitchPx);
        ctx.stroke();

        ctx.font = '500 10px "DM Mono", monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';

        for (let deg = -90; deg <= 90; deg += 5) {
            if (deg === 0) continue;
            const y = pitchPx - deg * PPD;
            if (y < -h || y > h) continue;

            const isMajor = deg % 10 === 0;
            const halfW = isMajor ? 40 : 20;

            ctx.strokeStyle = c.pitch;
            ctx.lineWidth = isMajor ? 1.2 : 0.7;
            ctx.beginPath();
            ctx.moveTo(-halfW, y);
            ctx.lineTo(halfW, y);
            ctx.stroke();

            if (isMajor) {
                ctx.fillStyle = c.pitchText;
                ctx.fillText(Math.abs(deg).toString(), halfW + 16, y);
                ctx.fillText(Math.abs(deg).toString(), -halfW - 16, y);
            }
        }

        if (targetPitch !== null && targetRoll !== null) {
            ctx.save();
            ctx.rotate(roll);
            ctx.rotate(-targetRoll);
            const tPitchPx = (targetPitch * RAD) * PPD;
            ctx.setLineDash(GHOST_DASH);
            ctx.strokeStyle = c.ghost;
            ctx.lineWidth = 2;
            // Ghost horizon
            ctx.beginPath();
            ctx.moveTo(-50, tPitchPx);
            ctx.lineTo(-12, tPitchPx);
            ctx.moveTo(12, tPitchPx);
            ctx.lineTo(50, tPitchPx);
            ctx.stroke();
            // Ghost wings
            ctx.beginPath();
            ctx.moveTo(-50, tPitchPx);
            ctx.lineTo(-50, tPitchPx + 8);
            ctx.moveTo(50, tPitchPx);
            ctx.lineTo(50, tPitchPx + 8);
            ctx.stroke();
            ctx.setLineDash([]);
            ctx.restore();
        }

        ctx.restore();

        // --- Bank Marks (drawn in screen space) ---
        ctx.save();
        ctx.translate(cx, cy);
        const bankR = Math.min(w, h) * 0.42;

        const bankAngles = [10, 20, 30, 45, 60];
        ctx.strokeStyle = c.bank;

        for (const ang of bankAngles) {
            for (const sign of [-1, 1]) {
                const a = (-90 + ang * sign) * DEG;
                const isMajor = ang === 30 || ang === 60;
                const len = isMajor ? 10 : 6;
                ctx.lineWidth = isMajor ? 1.2 : 0.8;
                ctx.beginPath();
                ctx.moveTo(Math.cos(a) * bankR, Math.sin(a) * bankR);
                ctx.lineTo(Math.cos(a) * (bankR - len), Math.sin(a) * (bankR - len));
                ctx.stroke();
            }
        }

        // Center mark (top)
        ctx.fillStyle = c.horizon;
        ctx.beginPath();
        ctx.moveTo(0, -bankR);
        ctx.lineTo(-5, -bankR + 8);
        ctx.lineTo(5, -bankR + 8);
        ctx.closePath();
        ctx.fill();

        ctx.save();
        ctx.rotate(-roll);
        ctx.fillStyle = c.aircraft;
        ctx.beginPath();
        ctx.moveTo(0, -bankR + 10);
        ctx.lineTo(-5, -bankR + 18);
        ctx.lineTo(5, -bankR + 18);
        ctx.closePath();
        ctx.fill();
        ctx.restore();

        ctx.restore();

        // --- Fixed Aircraft Symbol ---
        ctx.save();
        ctx.translate(cx, cy);
        ctx.strokeStyle = c.aircraft;
        ctx.lineWidth = 2.5;
        ctx.lineCap = 'round';

        // Wings
        ctx.beginPath();
        ctx.moveTo(-55, 0);
        ctx.lineTo(-18, 0);
        ctx.lineTo(-18, 6);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(55, 0);
        ctx.lineTo(18, 0);
        ctx.lineTo(18, 6);
        ctx.stroke();

        // Center dot
        ctx.fillStyle = c.aircraft;
        ctx.beginPath();
        ctx.arc(0, 0, 3, 0, Math.PI * 2);
        ctx.fill();

        ctx.restore();

        // --- Numeric readouts ---
        ctx.save();
        ctx.font = '500 10px "DM Mono", monospace';
        ctx.textBaseline = 'bottom';
        ctx.fillStyle = c.readout;

        ctx.textAlign = 'left';
        ctx.fillText('R ' + rollDeg.toFixed(1) + '\u00b0', 4, h - 3);
        ctx.textAlign = 'right';
        ctx.fillText('P ' + pitchDeg.toFixed(1) + '\u00b0', w - 4, h - 3);
        ctx.restore();
    }

    return { init, draw, resize };

})();
