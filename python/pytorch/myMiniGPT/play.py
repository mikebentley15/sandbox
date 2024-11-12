#!/usr/bin/env python3
import torch
import torch.nn as nn
from torch.nn import functional as F

torch.manual_seed(1337)
B, T, C = 4, 8, 2 # batch, time, channels
x = torch.randn(B, T, C)
print(x.shape)
print(x)

def bow1(x):
    '''Bag of words.  Returns xbow[b,t] = mean_{i<=t} x[b,i]'''
    B, T, C = x.shape
    xbow = torch.zeros((B,T,C))
    for b in range(B):
        for t in range(T):
            xprev = x[b, :t+1] # (t, C)
            xbow[b, t] = torch.mean(xprev, 0)
    return xbow

def bow2(x):
    '''Bag of words.  Returns xbow[b,t] = mean_{i<=t} x[b,i]'''
    B, T, C = x.shape
    W = torch.tril(torch.ones(T, T))
    W = W / torch.sum(W, 1, keepdim=True)
    xbow = W @ x # (T, T) @ (B, T, C) --> (B, T, C)
    return xbow

def bow3(x):
    '''Bag of words.  Returns xbow[b,t] = mean_{i<=t} x[b,i]'''
    B, T, C = x.shape
    tril = torch.tril(torch.ones(T, T))
    W = torch.zeros((T, T))
    W = W.masked_fill(tril == 0, float('-inf'))
    W = F.softmax(W, dim=-1)
    xbow = W @ x
    return xbow

print(f'bow2(x) == bow1(x) ? {torch.allclose(bow1(x), bow2(x))}')
print(f'bow3(x) == bow1(x) ? {torch.allclose(bow1(x), bow3(x))}')
