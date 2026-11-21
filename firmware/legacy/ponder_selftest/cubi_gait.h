// Cubi gait + reflex, trained in simulation by cubi_train.py. Re-run `python cubi_train.py export` after training.
#pragma once
#define CUBI_AMP_DEG   12.000f   // swing amplitude (+-)
#define CUBI_FREQ_HZ   2.200f   // step cycles per second
#define CUBI_FAST      0.348f   // fraction of the cycle swinging forward fast
#define CUBI_PH_LF     0.000f
#define CUBI_PH_LB     0.410f   // leg phase offsets (cycles)
#define CUBI_PH_RF     0.500f
#define CUBI_PH_RB     -0.088f
#define CUBI_TRIM_F    1.593f   // front / back leg centre offsets (deg)
#define CUBI_TRIM_B    -1.634f
#define CUBI_SMOOTH    0.000f   // rounds off leg reversals (0 = sharp sawtooth, 1 = minimum-jerk)
#define CUBI_EDGE_RATIO 1.35f  // ToF > ratio x flat-desk reading = edge
#define CUBI_WALL_RATIO 0.75f
#define CUBI_BACKUP_S   1.50f
