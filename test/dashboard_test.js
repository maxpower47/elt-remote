import test from 'node:test';
import assert from 'node:assert/strict';

// --- Dashboard Logic Functions Under Test ---

function voltageToPercent(v) {
    if (!v || v <= 3.3) return 0;
    if (v >= 4.2) return 100;
    if (v >= 4.0) return Math.round(80 + ((v - 4.0) / 0.2) * 20);
    if (v >= 3.8) return Math.round(55 + ((v - 3.8) / 0.2) * 25);
    if (v >= 3.7) return Math.round(35 + ((v - 3.7) / 0.1) * 20);
    if (v >= 3.6) return Math.round(15 + ((v - 3.6) / 0.1) * 20);
    return Math.round(((v - 3.3) / 0.3) * 15);
}

function getCapCellGrid(lat, lon) {
    if (!lat && !lon) return "--";
    const aLat = Math.abs(lat);
    const aLon = Math.abs(lon);
    if (aLat === 0 && aLon === 0) return "--";

    const baseLat = Math.floor(aLat);
    const baseLon = Math.floor(aLon);

    const latStr = baseLat.toString().padStart(2, '0');
    const lonStr = baseLon >= 100 ? baseLon.toString() : baseLon.toString().padStart(2, '0');
    const base = `${latStr}${lonStr}`;

    const latMin = (aLat - baseLat) * 60.0;
    const lonMin = (aLon - baseLon) * 60.0;

    function getQuad(lMin, loMin, span) {
        const isNorth = (lMin % span) >= (span / 2.0);
        const isWest = (loMin % span) >= (span / 2.0);
        if (isNorth && isWest) return 'A';  // NW
        if (isNorth && !isWest) return 'B'; // NE
        if (!isNorth && isWest) return 'C'; // SW
        return 'D';                         // SE
    }

    const q1 = getQuad(latMin, lonMin, 60.0);
    const q2 = getQuad(latMin, lonMin, 30.0);
    const q3 = getQuad(latMin, lonMin, 15.0);

    return `${base}${q1}${q2}${q3}`;
}

function decimalToDMS(val, isLat) {
    const dir = isLat ? (val >= 0 ? 'N' : 'S') : (val >= 0 ? 'E' : 'W');
    val = Math.abs(val);
    const deg = Math.floor(val);
    const remainderMins = (val - deg) * 60;
    const mins = Math.floor(remainderMins);
    const secs = (remainderMins - mins) * 60;
    return `${deg}°${mins.toString().padStart(2, '0')}'${secs.toFixed(1).padStart(4, '0')}"${dir}`;
}

// Depth-counted BLE packet reassembler (from docs/index.html)
function parseBleStream(chunk) {
    const packets = [];
    let startIdx = -1;
    let depth = 0;

    for (let i = 0; i < chunk.length; i++) {
        if (chunk[i] === '{') {
            if (depth === 0) startIdx = i;
            depth++;
        } else if (chunk[i] === '}') {
            if (depth > 0) {
                depth--;
                if (depth === 0 && startIdx !== -1) {
                    let jsonCandidate = chunk.substring(startIdx, i + 1);
                    jsonCandidate = jsonCandidate.replace(/[\u0000-\u001F\u007F-\u009F]/g, "");
                    packets.push(JSON.parse(jsonCandidate));
                    startIdx = -1;
                }
            }
        }
    }
    return packets;
}

// --- Unit Tests ---

test('JS LiPo Voltage-to-Percent Curve', () => {
    assert.equal(voltageToPercent(4.20), 100);
    assert.equal(voltageToPercent(4.35), 100);
    assert.equal(voltageToPercent(4.00), 80);
    assert.equal(voltageToPercent(3.80), 55);
    assert.equal(voltageToPercent(3.70), 35);
    assert.equal(voltageToPercent(3.60), 15);
    assert.equal(voltageToPercent(3.30), 0);
    assert.equal(voltageToPercent(2.80), 0);
});

test('JS 3-Level CAP Cell Grid Calculations', () => {
    assert.equal(getCapCellGrid(34.0522, -118.2437), '34118DDC');
    assert.equal(getCapCellGrid(40.75, -86.25), '4086BAD');
    assert.equal(getCapCellGrid(0, 0), '--');
    assert.equal(getCapCellGrid(null, null), '--');
});

test('JS DMS Coordinate Formatter', () => {
    const latDms = decimalToDMS(34.0522, true);
    assert.match(latDms, /^34°03'07\.\d"N$/);

    const lonDms = decimalToDMS(-118.2437, false);
    assert.match(lonDms, /^118°14'37\.\d"W$/);
});

test('JS BLE Stream Depth-Counted Multi-Packet Slicer', () => {
    const multiPacketStream = 'PREFIX_GARBAGE\x00{"type":"TELEMETRY","state":"DISARMED"}{"type":"TELEMETRY","state":"ACTIVE"}TRAILING_NOISE';
    const parsed = parseBleStream(multiPacketStream);

    assert.equal(parsed.length, 2);
    assert.equal(parsed[0].state, 'DISARMED');
    assert.equal(parsed[1].state, 'ACTIVE');
});
