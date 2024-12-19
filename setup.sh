if [ "$(uname)" = "Darwin" ]; then
  pip install open3d==0.15.1
elif [ "$(uname)" = "Linux" ]; then
  pip install -U open3d
fi

sudo apt install libc++-dev libc++abi-dev libcgal-dev libomp-dev -y

pip install -U numpy tqdm gradio plotly opencv-python

mkdir ssl
cd ssl
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -sha256 -days 365 -nodes -batch
