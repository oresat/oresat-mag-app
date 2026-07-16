dom 26 abr 2026 04:39:18 UTC

https://pdxaerospace.slack.com/archives/C0183EL5BPD/p1777128120587009

Ted Havelka
  Yesterday at 2:42 PM
In the repo 'oresat-firmware' I find board files for the MCXN protocard in oresat-firmware/apps/mcxn947-demo/boards/mcxn947_protocard.  While I am working on mag card firmware (still at early stages) I have an MCXN protocard and not the more specific mag card.  I've started my work in oresat-firmware/apps/mag but need to reference the protocard board files.  This is primarily for initial development work until I have access to a mag card.  Would a reasonable short term practice be for me to copy the protocard board files over into the apps/mag subdirectory?
￼
https://pdxaerospace.slack.com/archives/C0183EL5BPD/p1777136330207949?thread_ts=1777128120.587009&cid=C0183EL5BPD
￼
￼Pete Skeggs
  Yesterday at 4:58 PM
Ted, when I named that set of board files in apps/mcxn947-demo (based on Blen's work), I used the wrong name. It isn't the protocard, it's the breakout card. The MCXN protocard you have is different. At the time, the MCXN protocard did not yet exist except maybe in Andrew's head.
In the apps/template project, I will be merging my work based on Paul's MCUboot work. This new stuff includes a proper board file folder for the MCXN protocard, and creates a separate set of files for the MCXN breakout board based on the older, wrongly-named MCXN protocard files you found.
My branch is here, if you want to take a look: https://github.com/oresat/oresat-template-app/tree/feature-protocard.
Bottom line: please do not use oresat-firmware/apps/mcxn947-demo/boards/mcxn947_protocard for your protocard!
You can make a copy to your app for now of the files I will merge to the template app , but we will eventually likely move this board file folder, and the breakout board's, to the central oresat-firmware/zephyr branch (which gets deployed in your workspace in the /common folder). This will be a second location from which west can pull board files. This will make it easier to share board files between projects without copying them. (edited) 
￼
￼
￼Ted Havelka
  Yesterday at 8:57 PM
Thank you Pete.  I'm copying these notes to my local workdir.  I recall now some mention from you in the past couple weeks about the new board files, so I'll be sure to use those.


