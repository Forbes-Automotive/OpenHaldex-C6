# Third-Party Notices

OpenHaldex-C6 is distributed under the **Forbes Automotive Source-Available License
(FASL) v1.0** for the portions original to Forbes Automotive.

It also incorporates and derives from third-party work that is licensed
separately. Those portions retain their original licenses. Where a conflict exists
between FASL and an upstream license, **the upstream license governs the upstream
portions.** In particular, MIT-licensed portions remain usable under the MIT terms
(including commercial use) regardless of FASL.

This file records the upstream projects, the components derived from them, and the
license/permission basis relied upon.

---

## 1. Original OpenHaldex (Generation 1)

- **Author / Upstream:** ABangingDonk
- **Project:** OpenHaldex / OpenHaldexT4 — https://github.com/ABangingDonk/OpenHaldexT4
- **Relationship:** OpenHaldex-C6 started from ABangingDonk's original OpenHaldex
  codebase for Generation 1 Haldex control.

---

## 2. OpenHaldex-S3 (SpringfieldVW / Chris "meatro")

- **Author / Upstream:** Chris (GitHub: meatro)
- **Project:** OpenHaldex-S3 — https://github.com/meatro/OpenHaldex-S3
- **License:** MIT License (see full text below)
- **Relationship:** On 2026-02-08, commit `a4dc321` ("S3 port onto C6") imported
  work derived from OpenHaldex-S3 into this repository. Derived / ported components
  include, in whole or in part:
  - CAN analysis / GVRET / SavvyCAN-style tooling (CAN View)

> MIT requires that the copyright notice and permission notice below be retained in
> all copies or substantial portions of the derived code. These portions remain
> available under MIT and are not restricted by FASL.

### MIT License (OpenHaldex-S3)

```
MIT License

Copyright (c) OpenHaldex-S3 contributors (SpringfieldVW / Chris "meatro")

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 3. Forbes Automotive Original Work

- Portions original to Forbes Automotive (including Gen2 / Gen4 / Gen5 reverse
  engineering and implementation, other Forbes-authored code, and the hardware
  design files — Gerbers, schematics, PCB layouts and enclosure files) are
  distributed under the **Forbes Automotive Source-Available License (FASL) v1.0**.
  See `LICENSE.md`.

---

## Summary of licensing model

- **Project labeling:** OpenHaldex-C6 is **source-available**, not "open source" in
  the OSI sense. FASL restricts commercial use and redistribution.
- **MIT portions** (OpenHaldex-S3 derived, and any MIT-basis upstream) remain under
  MIT and may be used commercially by others under MIT terms.
- **Attribution:** upstream copyright and permission notices must be preserved in
  source and binary distributions, per both MIT and FASL redistribution terms.
