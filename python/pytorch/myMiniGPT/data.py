#!/usr/bin/env python3

import requests
import sys
import logging
import os
import torch

import util

def main(arguments):
    util.setup_logging()

    sp_url = 'https://raw.githubusercontent.com/karpathy/char-rnn/master/data/tinyshakespeare/input.txt'
    sp_file = 'tinyshakespeare.txt'
    if os.path.exists(sp_file):
        logging.info(f'Skipping download: File "{sp_file}" already exists')
    else:
        logging.info(f'Downloading "{sp_file}"')
        util.download(sp_url, sp_file)

    logging.info(f'Reading "{sp_file}"')
    text = util.get_text(sp_file)

    logging.debug(f'Dataset length (in characters): {len(text)}')
    chars = sorted(set(text))
    vocab_size = len(chars)
    logging.debug(f'Unique chars: {repr("".join(chars))} (length {vocab_size})')

    enc, dec = util.gen_encoder_decoder(chars)

    msg = 'Hello World!'
    emsg = enc(msg)
    iemsg = dec(emsg)
    logging.info(f'Encoding of "{msg}": {emsg}')
    logging.info(f'Decoding of {emsg}: "{iemsg}"')

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
