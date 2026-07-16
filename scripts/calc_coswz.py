#!/usr/bin/env python3

import math

# *   COSWZ = cos(2*pi*fdesired/32)
# *
# * Step2:
# *   If abs(COSWZ)≤0.875
# *     NF_COSWZ = round[COSWZ*256]
# *     NF_COSWZ_SEL = 0
# *   else
# *     NF_COSWZ_SEL = 1
# *     if COSWZ > 0.875
# *       NF_COSWZ = round[8*(1-COSWZ)*256]
# *     else if COSWZ < 0.875
# *       NF_COSWZ = round[-8*(1+COSWZ)*256]
# *     end
# *   End


def main() -> None:

   fdesired = 1
   coswz = math.cos(2 * math.pi * fdesired / 32)
   if abs(coswz) <= 0.875:
       nf_coswz = round(coswz * 256, 3)
       nf_coswz_sel = 0
   else:
       nf_coswz_sel = 1
       if coswz > 0.875:
          nf_coswz = round(8 * (1 - coswz) * 256, 3)
       elif coswz < 0.875:
          nf_coswz = round(-8 * (1 + coswz) * 256, 3)

   print(f"For fdesired:{fdesired}KHz --> set nf_coswz_sel:{nf_coswz_sel}, nf_coswz:{nf_coswz}")


if __name__ == '__main__':
    main()
