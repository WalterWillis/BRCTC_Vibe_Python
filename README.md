# BRCTC_Vibe_Python
The original Python version of our project. It became deprecated when I learned the SPI library we were using did not function as expected.

The purpose of moving to Python from C/C++ was for the simplicity of code changes to testing. Most times, we make a small modification to our code and then test it. Doing this with C/C++ was a bit of a pain due to the heavy reliance on command line. It took time to SCP the files and then manually compile. However, using VS Code on the PI turned out to be very slow so the move wasn't really all that time-saving.

Another reason for moving to Python was the easy use of multi-threading. It didn't take long at all to get the multi-core threading down. Learning the Gyroscope logic was far more complicated. It took quite a bit of time to figure out that the Gyroscope byte values were inverted. The PI does not support changing to LSB first so I ended up having to simply invert the order of the byte pairs.

A JR member of our team helped with some of the code. His name is Justin Shay. He created most of the logging logic as well as the telemetry thread.
