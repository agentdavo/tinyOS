#!/usr/bin/env node
// SPDX-License-Identifier: MIT OR Apache-2.0
// Round-trip verifier for the machine editor's TSV parsers.
//
// We extract the <script> block from index.html, stub out the parts that
// assume a browser, and evaluate the result in a fresh Function() scope so
// we're testing the exact code that ships in the editor. Expect byte-
// identical output for the two shipped kinematic TSVs and for
// devices/embedded_toolpods.tsv.

"use strict";
const fs = require("fs");
const path = require("path");

const HTML = fs.readFileSync(path.join(__dirname, "index.html"), "utf8");

// Grab everything between the last <script> (no src=) and </script>.
const scripts = [...HTML.matchAll(/<script(?![^>]*src=)[^>]*>([\s\S]*?)<\/script>/g)];
if (!scripts.length) { console.error("no inline <script> found"); process.exit(2); }
const source = scripts[scripts.length - 1][1];

// Sandbox: no window, no document. The module-export tail returns the API.
const exported = new Function("module", source + "\nreturn module.exports;")({exports:{}});

// __dirname is the editor's directory (machines/). Repo root is one up.
const repo = path.resolve(__dirname, "..");

function checkKinematic(relpath) {
  const p = path.join(repo, relpath);
  const original = fs.readFileSync(p, "utf8");
  const parsed = exported.parseKinematicTSV(original);
  const out = exported.serializeKinematicTSV(parsed);
  const ok = out === original;
  console.log(`[${ok ? "ok" : "FAIL"}] ${relpath}  (${original.length} -> ${out.length} bytes)`);
  if (!ok) {
    const a = original.split("\n"), b = out.split("\n");
    for (let i = 0; i < Math.max(a.length, b.length); ++i) {
      if (a[i] !== b[i]) {
        console.log(`  line ${i+1} diff:`);
        console.log(`    orig: ${JSON.stringify(a[i])}`);
        console.log(`    out : ${JSON.stringify(b[i])}`);
        break;
      }
    }
  }
  return ok;
}

function checkPods(relpath) {
  const p = path.join(repo, relpath);
  const original = fs.readFileSync(p, "utf8");
  const parsed = exported.parsePodsTSV(original);
  const out = exported.serializePodsTSV(parsed);
  const ok = out === original;
  console.log(`[${ok ? "ok" : "FAIL"}] ${relpath}  (${original.length} -> ${out.length} bytes)`);
  if (!ok) {
    const a = original.split("\n"), b = out.split("\n");
    for (let i = 0; i < Math.max(a.length, b.length); ++i) {
      if (a[i] !== b[i]) {
        console.log(`  line ${i+1} diff:`);
        console.log(`    orig: ${JSON.stringify(a[i])}`);
        console.log(`    out : ${JSON.stringify(b[i])}`);
        break;
      }
    }
  }
  return ok;
}

function checkForwardKinematics() {
  // Load mill3 and verify Z axis at position=100 is 100 above Y origin + Z offset.
  const text = fs.readFileSync(path.join(repo, "machines/kinematic_mill3.tsv"), "utf8");
  const chain = exported.parseKinematicTSV(text);
  const zIdx = chain.axes.findIndex(a => a.name === "Z");
  if (zIdx < 0) { console.log("[FAIL] no Z axis"); return false; }
  chain.axes[zIdx].position = 100;
  const { world } = exported.computeForwardKinematics(chain);
  const wz = world[zIdx];
  // World origin of Z = (12,13,14) of its world matrix, should be (0, 0, 100)
  // since X=Y=0, Z offset is 0, dir is (0,0,1), position=100.
  const ox = wz[12], oy = wz[13], oz = wz[14];
  const ok = Math.abs(ox) < 1e-5 && Math.abs(oy) < 1e-5 && Math.abs(oz - 100) < 1e-5;
  console.log(`[${ok ? "ok" : "FAIL"}] forward kinematics: Z@100 -> (${ox.toFixed(3)}, ${oy.toFixed(3)}, ${oz.toFixed(3)})`);
  return ok;
}

let pass = 0, fail = 0;
for (const r of [
  checkKinematic("machines/kinematic_mill3.tsv"),
  checkKinematic("machines/kinematic_millturn.tsv"),
  checkKinematic("machines/kinematic_mx850.tsv"),
  checkPods("devices/embedded_toolpods.tsv"),
  checkForwardKinematics(),
]) {
  if (r) ++pass; else ++fail;
}
console.log(`\n${pass} passed, ${fail} failed.`);
process.exit(fail ? 1 : 0);
