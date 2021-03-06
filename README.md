LoonReader is an open eBook reader which automatically syncs its software and
content to a GitHub repo.

* Open hardware - based on the Arduino Uno board
* Open source code (in Wiring, the language used to program Arduino)
* Content updated directly from GitHub
* 2.8" TFT Touch Shield (includes microSD card holder) from Adafruit

Currently you can read text files off of an SD card.

The basic reader will have:

* A 'code' branch, an Arduino sketch which controls the device
* A 'content' branch, which contains text to be stored on the microSD card
* A 'sync' branch, with a script to upload the latest code and content to the LoonReader

Future steps:

* Sync script instead of separate read/write programs
* Add Flora GPS Shield from Adafruit, for:
** Controlling content by sun, stars, <a href="https://github.com/mapmeld/moon-phase">phase of the moon</a>
** Location-based stories and games
** Travel guides
** Location-logging

Acquiring Content:

Making the content open-source allows people to recommend and add content through pull
requests. It does, unfortunately, raise issues about copyright (I won't accept pull
requests with non-free content) and the element of surprise (location-based stories may
lose some mystery with open source, but you could always create your own story, use ROT13
and add a decoder to the device, give the device to a friend or a child, or NOT PEEK).
