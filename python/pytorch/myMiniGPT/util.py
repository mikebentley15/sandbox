import logging

import torch

DEFAULT_BLOCK_SIZE = 8
DEFAULT_BATCH_SIZE = 4
DEVICE = 'cpu'

def setup_logging():
    logging.basicConfig(level=logging.DEBUG)

def log_variables(level=logging.DEBUG, **kwargs):
    for key, val in kwargs.items():
        logging.log(level=level, msg=f'{key}: {val}')

def download(url, dest):
    import requests
    gotten = requests.get(url)
    with open(dest, 'wb') as fout:
        fout.write(gotten.content)

def gen_encoder_decoder(chars):
    char_map = {ch: i for i, ch in enumerate(chars)}
    enc = lambda s: [char_map[c] for c in s]
    dec = lambda e: ''.join(chars[i] for i in e)
    return (enc, dec)

def get_text(filename='tinyshakespeare.txt'):
    with open(filename, 'r') as fin:
        text = fin.read()
    return text

def get_batch(data,
              block_size=DEFAULT_BLOCK_SIZE,
              batch_size=DEFAULT_BATCH_SIZE):
    ix = torch.randint(len(data) - block_size, (batch_size,))
    x = torch.stack([data[i:i+block_size] for i in ix])
    y = torch.stack([data[i+1:i+block_size+1] for i in ix])
    x, y = x.to(DEVICE), y.to(DEVICE)
    return (x, y)

def set_seed(seed=1337):
    torch.manual_seed(seed)
