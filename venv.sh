sudo apt install -y python3.12-venv
python3 -m venv ~/SnekBot/venv
source ~/SnekBot/venv/bin/activate
pip install --upgrade pip
pip install setuptools
pip install roboticstoolbox-python
pip install "numpy<2"

sudo usermod -aG dialout $USER
newgrp dialout
