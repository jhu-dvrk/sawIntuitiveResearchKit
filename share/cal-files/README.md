# Nominal MTM Calibration Files (`MTML-average.cal` and `MTMR-average.cal`)

If you have the original factory `.cal` files from Intuitive Surgical for your arms, **you should always use them**.

When original `.cal` files cannot be located for Master Tool Manipulators (MTMs), you can use the nominal calibration files in this directory (`MTML-average.cal` for a Left MTM and `MTMR-average.cal` for a Right MTM) to generate your initial dVRK IO configuration files.

> **Important:** Left (`MTML`) and Right (`MTMR`) `.cal` files are different and **must not** be swapped.

---

## How the Nominal Files Were Computed

The values in `MTML-average.cal` and `MTMR-average.cal` were computed as the mean of the MTM `.cal` files in [`dvrk_config_jhu`](https://github.com/dvrk-config/dvrk_config_jhu):

* **`MTML` (N = 3 unique arms):**
  * `jhu-dVRK/cal-files/jhu-mtml-m22723.cal`
  * `jhu-dVRK/cal-files/jhu-mtml-m28003.cal`
  * `jhu-daVinci/cal-files/jhu-dv-mtml-34863.cal`
* **`MTMR` (N = 3 unique arms):**
  * `jhu-dVRK/cal-files/jhu-mtmr-m28247.cal`
  * `jhu-daVinci/cal-files/jhu-dv-mtmr-32336.cal`
  * `jhu-daVinci-Si/cal-files/jhu-mtmr-22403.cal`

### `MTML` Statistics (N = 3, Mean ± SD)

```text
Act  pot_input_gain (rad/bit)  IO Scale (rad/V)   CV     pot_input_offset (rad)  IO Offset (deg)
---  ------------------------  -----------------  -----  ----------------------  -----------------
 0   +0.0014577 ± 0.0000031    +1.3268 ± 0.0028   0.21%  -2.640839 ± 0.12854     -151.31° ±  7.37°
 1   +0.0014480 ± 0.0000069    +1.3180 ± 0.0063   0.48%  -2.433938 ± 0.14150     -139.45° ±  8.11°
 2   +0.0014507 ± 0.0000045    +1.3204 ± 0.0041   0.31%  -2.691767 ± 0.17320     -154.23° ±  9.92°
 3   -0.0014650 ± 0.0000061    -1.3335 ± 0.0055   0.42%  +1.956299 ± 0.16923     +112.09° ±  9.70°
 4   -0.0015223 ± 0.0000021    -1.3857 ± 0.0019   0.14%  +3.913745 ± 0.04481     +224.24° ±  2.57°
 5   +0.0015067 ± 0.0000070    +1.3714 ± 0.0064   0.47%  -3.115468 ± 0.08559     -178.50° ±  4.90°
 6   +0.0007697 ± 0.0000029    +0.7006 ± 0.0026   0.38%  -0.298689 ± 1.56602      -17.11° ± 89.73°
```

### `MTMR` Statistics (N = 3, Mean ± SD)

```text
Act  pot_input_gain (rad/bit)  IO Scale (rad/V)   CV     pot_input_offset (rad)  IO Offset (deg)
---  ------------------------  -----------------  -----  ----------------------  -----------------
 0   +0.0014548 ± 0.0000052    +1.3242 ± 0.0047   0.36%  -3.185465 ± 0.06464     -182.51° ±  3.70°
 1   +0.0014423 ± 0.0000069    +1.3128 ± 0.0063   0.48%  -2.532009 ± 0.02400     -145.07° ±  1.37°
 2   +0.0014515 ± 0.0000120    +1.3212 ± 0.0110   0.83%  -2.793417 ± 0.11274     -160.05° ±  6.46°
 3   -0.0014678 ± 0.0000014    -1.3360 ± 0.0013   0.09%  +3.834052 ± 0.16487     +219.67° ±  9.45°
 4   +0.0015076 ± 0.0000059    +1.3723 ± 0.0054   0.39%  -2.348690 ± 0.06876     -134.57° ±  3.94°
 5   +0.0015169 ± 0.0000160    +1.3808 ± 0.0146   1.06%  -3.076881 ± 0.12639     -176.29° ±  7.24°
 6   +0.0007711 ± 0.0000038    +0.7019 ± 0.0035   0.50%  -0.579371 ± 1.72870      -33.20° ± 99.05°
```
