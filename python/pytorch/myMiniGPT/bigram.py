#!/usr/bin/env python3

import torch
import torch.nn as nn
from torch.nn import functional as F
import sys
import logging

import util

class BigramLanguageModel(nn.Module):
    def __init__(self, vocab_size):
        super().__init__()
        # each token directly reads off the logits for the next token
        self.token_embedding_table = nn.Embedding(vocab_size, vocab_size)

    def forward(self, idx, targets=None):
        '''
        idx and targets are both (B,T) tensor of integers.  Returns logits of
        size (B,T,C).
        - B: batch size
        - T: time (i.e., block_size)
        - C: channels (i.e., vocab_size)
        '''
        logits = self.token_embedding_table(idx) # (B,T,C)
        loss = None
        if targets is not None:
            B, T, C = logits.shape
            loss_logits = logits.view(B*T, C) # (BT, C)
            targets = targets.view(B*T)  # (BT,)
            loss = F.cross_entropy(loss_logits, targets)
        return logits, loss

    def generate(self, idx, max_new_tokens):
        '''
        Generate next max_new_tokens new tokens and append to idx.

        idx is (B, T) array of indices in the current context
        '''
        for _ in range(max_new_tokens):
            logits, loss = self(idx) # get the predictions
            logits = logits[:, -1, :] # focus only on the last time step
            probs = F.softmax(logits, dim=-1) # apply softmax to get probabilities
            idx_next = torch.multinomial(probs, num_samples=1) # sample from probs
            idx = torch.cat((idx, idx_next), dim=1) # append generated to idx
        return idx

@torch.no_grad()
def estimate_loss(model, train, val, eval_iters, batch_size, block_size):
    'Estimate training and validation loss as a dictionary'
    out = {}
    model.eval()
    split_mapping = {'train': train, 'val': val}
    for split, data in split_mapping.items():
        losses = torch.zeros(eval_iters)
        for k in range(eval_iters):
            X, Y = util.get_batch(data, batch_size=batch_size, block_size=block_size)
            logits, loss = model(X, Y)
            losses[k] = loss.item()
        out[split] = losses.mean()
    model.train()
    return out

def main(arguments):
    # hyperparams
    batch_size = 32
    block_size = 8
    max_iters = 3001
    eval_interval = 300
    learning_rate = 1e-2
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    eval_iters = 200
    train_ratio = 0.9
    seed = 1337

    util.setup_logging()
    util.set_seed(seed)
    util.DEVICE = device
    util.log_variables(
        level=logging.INFO,
        batch_size=batch_size,
        block_size=block_size,
        max_iters=max_iters,
        eval_interval=eval_interval,
        learning_rate=learning_rate,
        device=device,
        eval_iters=eval_iters,
        train_ratio=train_ratio,
        seed=seed,
    )

    text = util.get_text()
    chars = sorted(set(text))
    vocab_size = len(chars)
    encode, decode = util.gen_encoder_decoder(chars)
    data = encode(text)
    N_train = int(train_ratio * len(data))
    train = torch.tensor(data[:N_train])
    val = torch.tensor(data[N_train:])

    m = BigramLanguageModel(vocab_size).to(device)
    xb, yb = util.get_batch(train, batch_size=batch_size, block_size=block_size)
    logits, loss = m(xb, yb)
    logging.debug(f'logits.shape: {logits.shape}')
    logging.debug(f'loss:         {loss}')

    # generate
    def create_text(model, size):
        context = torch.zeros((1, 1), dtype=torch.long, device=device)
        created = model.generate(context, max_new_tokens=size)
        return decode(created[0].tolist())
    logging.debug(f'created text before training: {repr(create_text(m, size=100))}')

    # train
    optimizer = torch.optim.AdamW(m.parameters(), lr=learning_rate)
    for steps in range(max_iters):
        xb, yb = util.get_batch(train, batch_size=batch_size, block_size=block_size)
        logits, loss = m(xb, yb)
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        optimizer.step()

        if steps % eval_interval == 0:
            estimate = estimate_loss(m, train, val, eval_iters, batch_size, block_size)
            logging.debug(f'{steps:5d} loss: {estimate}')

    logging.debug(f'created text after training:\n{create_text(m, size=500)}')

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
