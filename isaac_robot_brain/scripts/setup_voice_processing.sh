#!/bin/bash
# setup_voice_processing.sh
# Script to set up voice processing components for Vision-Language-Action module

set -e  # Exit on any error

echo "Setting up voice processing components..."

# Install audio processing dependencies
echo "Installing audio processing dependencies..."
pip3 install pyaudio speechrecognition webrtcvad sounddevice

# Download Whisper model if not already present
echo "Downloading Whisper model..."
python3 -c "
import whisper
print('Loading Whisper model...')
model = whisper.load_model('base')
print('Whisper model loaded successfully.')
"

# Set up audio configuration
echo "Configuring audio settings..."
if [ ! -f ~/.asoundrc ]; then
    echo "Creating audio configuration file..."
    cat > ~/.asoundrc << EOF
pcm.!default {
    type pulse
}
ctl.!default {
    type pulse
}
EOF
fi

echo "Voice processing setup completed successfully!"