# Prometheus: Chess Engine
*<p style="text-align: center;">October 31, 2021</p>*

## Introduction
I go over how classical chess engines work, describe my own chess engine, [Prometheus](https://github.com/zborffs/Prometheus), and discuss the results of some experiments that guided me in the design thereof.

If you want to learn more about chess engines, I recommend checking out [this site](https://chessprogramming.org/). 

### Demo
![Demonstration of Engine Output](prometheus-images/senpai_terminal.gif)
*<p style="text-align: center;">Figure 1: Demo of me using UCI commands to make Prometheus perform a search.</p>*

## Background

### What is a chess engine?
Chess engines are computer programs that compute the best move from a given chess position.

![Chess Position]()
*<p style="text-align: center;">Figure 2: Position</p>*

### How do chess engines work?
At a high-level, chess engines go about determining (what it _believes_ to be) the best move by generating every possible combination of legal moves stemming from the given position, assigning values to the final positions precipitating from each move combination, then "bubbling up" those scores back to the starting position to determine what combination of moves should be made to acheive the best possible position.

If you want detail about each one of those steps, I highly recommend checking out [this site](https://chessprogramming.org/).

## Prometheus
Prometheus is a chess engine written in C++. 

![ELO TABLE]()
Tournament tests with Senpai, Stockfish CPW place it's ELO at around 2900.

### Move Generation

### Tranposition Table

### Search Features

### Evaluation Features

### Evaluation Parameter Tuning
