#!/bin/sh
python3 aprs_wav_generator.py \
  --source N0CALL-9 \
  --dest APRS \
  --path WIDE1-1,WIDE2-1 \
  --info ">Hello from Python" \
  --output aprs.wav