# Prometheus: Chess Engine
*<p style="text-align: center;">October 31, 2021</p>*

## Introduction
I provide a high-level overview of how classical chess engines work, describe my own chess engine, [Prometheus](https://github.com/zborffs/Prometheus), and discuss the results of several experiments that guided me in the design thereof.

If you want to learn more about chess engines or make your own, I *highly* recommend checking out [this site](https://chessprogramming.org/). 

### Demo
![Demonstration of Engine Output](prometheus-images/senpai_terminal.gif)
*<p style="text-align: center;">Figure 1: Demo of me using UCI commands to make Prometheus perform a search.</p>*

## Background
Chess engines are computer programs that compute the best move from a given chess position.

![Input-Output block diagram of Chess Engines]()
*<p style="text-align: center;">Figure 2: Input-output block diagram of chess engines.</p>*

At a high-level, chess engines go about determing what it *believes* to be the best move by: (1) generating every possible combination of legal moves stemming from the given position, (2) assigning values to the final positions precipitating from the previous step, which correspond to the position's relative strength, and (3) "bubbling up" those scores back to the starting position in such a manner as to assume that the opponent will always make what the engine believes is the best move available to it. 

In actuality, even the simplest of chess engines don't do that exactly. My discussion of the different experiments and design choices in the subsequent sections should help illustrate the subtleties of how classical chess engines actually work.


## Prometheus
Prometheus is a chess engine I wrote in C++.

The following table provides a summary of the overall performance of Prometheus against several other open-source chess engines as a result of 5000 matches played at various time controls. From these results, we can compute Promethues's ELO to be roughly 2900. 

![ELO Table]()
*<p style="text-align: center;">Figure 3: ELO table of tournament.</p>*

Chess engines, in my view, consist of three main modules: a move generation module, a search module, and an evaluation module. In the following sections, I will describe each module and go over the results of several experiments I conducted to inform design decisions within that module.

## Move Generation
The move generation module is responsible for generating all the legal moves stemming from a given position as efficiently as possible. Efficiency in this context is measured by (1) *how quickly we can generate moves on average*, and (2) *how much space moves take up in memory*. 

The reason that these metrics are chosen is because, the search module will call to this module (literally) millions of times for a single search of the game tree. Therefore, small improvements in the average move generation computation time or small improvements in the amount of memory that a single move takes up in memory will translate to massive improvements in the engine's overall performance.

I made use of [bitboards](https://www.chessprogramming.org/Bitboards) to represent positions, which enabled me to design my move generation algorithm in terms of bitwise operations. To generate sliding piece moves, I made of [BMI2](https://en.wikipedia.org/wiki/X86_Bit_manipulation_instruction_set) [magic bitboard](https://www.chessprogramming.org/Magic_Bitboards) to generate moves exceedingly quickly. Finally, I designed a movelist data structure for optimally storing moves and manipulating the order thereof.

### Bitboards

### BMI2 Magic Bitboards

### Move List


## Search
The search module is responsible for searching the game tree as efficiently as possible. Efficiency in this context refers to *how deep the search can get on average in a fixed period of time*, because the deeper the search, the more informed the ultimate decision of picking the best move is. 

For the most part, the search module will search roughly the same number of nodes per second regardless of its position within the game tree. In positions where the branching factor of the game tree is high (i.e. in the middle of the game), the depth that search will reach after a fixed period of time will be much lower than in positions where the branching factor is low (i.e. at the end of the game). Since the nodes searched per second is roughly constant, the design objectives for the search algorithm are to look at as few nodes as possible at a given depth. This is achieved through ***pruning*** branches of the tree containing move combinations that are likely to result in such bad outcomes for one or both sides that the probability of entering those parts of the game tree are small.

This accords well with how most humans play chess. Humans do not look at the outcome of every possible combination of moves. Instead, a human will immediately come up with 2 or 3 candidate moves at each depth they search to. The difference between a strong player and a weak player is probably in coming up with such candidate moves at gestalt. Dido with chess engines. The best chess engines will order the moves generated by the move generation module is such a way as to prune the rest of the game tree so consistently well, that they can achieve much deeper searches, and by extension make much better moves. 

### Tranposition Table

### Search Feature 1
### Search Feature 1
### Search Feature 1
### Search Feature 1

### Opening Book

## Evaluation
The evaluation module is responsible for assigning scores to positions in such a way as to maximize the number of wins. This is not to say the evaluation should seek to "accurately" score moves, because there is no way of measuring how "accurate" an evaluation function is, so it does not make sense to say we are trying to make the evaluation function more accurate. That being said, some positions are clearly better than other positions, so the evaluations of each position should reflect that. 

### Evaluation Feature 1

### Evaluation Feature 2

### Evaluation Function Parameter Tuning
