#include "app/PluginFunctions.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <iostream>
#include <random>
#include <ranges>
#include <stdexcept>
#include <string_view>
#include <bitset>

using State = PluginFunctions::State;

constexpr std::size_t wordLength{5};
constexpr std::size_t guessLimit{6};
using Word = char[wordLength + 1];
constexpr Word allowedWords[]{
#include "build/wordle_allowed_wrap.h"
};
constexpr Word answerWords[]{
#include "build/wordle_answers_wrap.h"
};
constexpr std::size_t allowedWordCount = sizeof(allowedWords) / sizeof(Word);
constexpr std::size_t answerWordCount = sizeof(answerWords) / sizeof(Word);

namespace colorcode {
constexpr auto boldGreen = "\033[1;32m";
constexpr auto boldYellow = "\033[1;33m";
// constexpr auto gray = "\033[2m";
constexpr auto gray = "\033[30m";
constexpr auto reset = "\033[0m";
constexpr auto normal = reset;
} // namespace colorcode

namespace {

auto isWordIn(const std::string_view word, const std::ranges::input_range auto& words) {
  return std::ranges::binary_search(words, word);
}

auto isWord(const std::string_view word) -> bool {
  return word.size() == wordLength
         && (isWordIn(word, allowedWords) || isWordIn(word, answerWords));
}

enum class LetterStatus : uint8_t {
  unknown,
  missing,
  wrongSpot,
  correctSpot,
};

auto getColorCode(const LetterStatus status) -> std::string_view {
  switch(status) {
    case LetterStatus::unknown:     return colorcode::normal;
    case LetterStatus::missing:     return colorcode::gray;
    case LetterStatus::wrongSpot:   return colorcode::boldYellow;
    case LetterStatus::correctSpot: return colorcode::boldGreen;
  }
  return colorcode::normal;
}

auto chooseRandomWord() -> const Word& {
  static std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<std::size_t> dist{0, answerWordCount - 1};
  return answerWords[dist(gen)];
}

struct WordleState {
  std::string_view correctWord{};
  Word guesses[guessLimit]{};
  std::size_t guessCount{0};
  bool victory{};

  static auto newGame() -> WordleState {
    return WordleState{.correctWord = chooseRandomWord()};
  }

  auto getLetterStatus(const std::string_view guess) const {
    std::array<LetterStatus, wordLength> letterStatus;
    letterStatus.fill(LetterStatus::missing);
    std::bitset<wordLength> usedLetters;

    // First find matching letters
    for (std::size_t i = 0; i < wordLength; ++i) {
      if (guess[i] == correctWord[i]) {
        usedLetters.set(i);
        letterStatus[i] = LetterStatus::correctSpot;
      }
    }

    // For remaining letters, see if there are matching letters in other spots
    for (std::size_t i = 0; i < wordLength; ++i) {
      if (letterStatus[i] == LetterStatus::correctSpot) { continue; }
      for (std::size_t j = 0; j < wordLength; ++j) {
        if (!usedLetters.test(j) && guess[i] == correctWord[j]) {
          usedLetters.set(j);
          letterStatus[i] = LetterStatus::wrongSpot;
          break;
        }
      }
    }

    return letterStatus;
  }

  auto accumulateLetterStatus() const -> std::array<LetterStatus, 26> {
    std::array<LetterStatus, 26> statusForEachLetter;
    statusForEachLetter.fill(LetterStatus::unknown);
    std::for_each(guesses, guesses + guessCount, [this, &statusForEachLetter](const std::string_view word) {
      for (std::size_t i = 0; i < wordLength; ++i) {
        const auto ch = word[i];
        const auto idx = ch - 'a';
        auto& status = statusForEachLetter[idx];
        if (LetterStatus::correctSpot == status || LetterStatus::missing == status) {
          // do nothing
        } else if (ch == correctWord[i]) {
          status = LetterStatus::correctSpot;
        } else if (std::string_view::npos != correctWord.find(ch)) {
          status = LetterStatus::wrongSpot;
        } else {
          status = LetterStatus::missing;
        }
      }
    });
    return statusForEachLetter;
  }

  void printStatus() const {
    // Print each word with colors
    std::for_each(guesses, guesses + guessCount, [this](const std::string_view word) {
      const auto letterStatus = getLetterStatus(word);
      fmt::print("\n  ");
      for (std::size_t i = 0; i < wordLength; ++i) {
        fmt::print("{}{}{} ", getColorCode(letterStatus[i]), word[i], colorcode::reset);
      }
    });

    // Print the alphabet with colors
    const auto statusForEachLetter = accumulateLetterStatus();
    fmt::print("\n\n  -----------------------");
    for (char i = 0; static_cast<std::size_t>(i) < statusForEachLetter.size(); ++i) {
      const char ch = 'a' + i;
      const auto letterStatus = statusForEachLetter[i];
      if (i % 10 == 0) {
        fmt::print("\n"
                   "    ");
      }
      fmt::print("{}{}{} ", getColorCode(letterStatus), ch, colorcode::reset);
    }
    fmt::print("        \n"
               "  -----------------------\n\n");

    // TODO: print keyboard with status of each letter
    if (victory) {
      fmt::print("{}You won in {} guesses!{}\n\n", colorcode::boldGreen, guessCount, colorcode::reset);
    } else {
      fmt::print("You have {} guesses left\n\n", guessLimit - guessCount);
    }
  }

  void makeGuess(const std::string_view guess) {
    constexpr auto restartMsg = "Restart (with 'wordle') if you want to play again.\n";
    if (victory) {
      std::puts("You already guessed the correct word.");
      std::puts(restartMsg);
      return;
    }

    if (isWordIn(guess, std::ranges::subrange(guesses, guesses + guessCount))) {
      std::puts("You already guessed that word.");
      return;
    }

    ++guessCount;
    if (guessCount > guessLimit) {
      std::puts("You're out of guesses.");
      std::puts(restartMsg);
      return;
    }

    // store guess
    std::memcpy(guesses[guessCount-1], guess.data(), wordLength);
    guesses[guessCount-1][wordLength] = '\0';
    victory = (guess == correctWord);

    printStatus();
    if (victory) {
      std::puts(restartMsg);
      return;
    }

    if (guessCount == guessLimit) {
      fmt::print("Sorry, the correct word was '{}'.\n", correctWord);
      std::puts(restartMsg);
      return;
    }
  }
};

auto toWordleState(State raw) -> WordleState* {
  return reinterpret_cast<WordleState*>(raw);
}

State pluginInit() {
  auto* const state = new WordleState{};
  *state = WordleState::newGame();
  return state;
}

void pluginPreReload(State) {
  // do nothing
}

void pluginPostReload(State) {
  // do nothing
}

void pluginDestroy(State rawState) {
  auto* const state = toWordleState(rawState);
  delete state;
}

void printHelp(State) {
  std::puts(
      "WordlePlugin:\n"
      "  Commands:\n"
      "    wordle          restart the wordle.\n"
      "    wordle-status   print out a status report with the current guesses.\n"
      "    wordle-cheat    print out the correct word.\n"
      "\n"
      "  Just type in a five letter word to get started until you're out of\n"
      "  guesses.\n"
  );
}

bool tryHandleInput(State rawState, const std::string_view input) {
  try {
    auto* const state = toWordleState(rawState);
    if (state == nullptr) {
      std::fputs("Error: Wordle state has not been initialized\n", stderr);
      return false;
    }
    if (input == "wordle") {
      fmt::print("Restarting wordle game.  {} guesses remaining.\n", guessLimit);
      *state = WordleState::newGame();
      return true;
    }
    if (input == "wordle-status") {
      state->printStatus();
      return true;
    }
    if (input == "wordle-cheat") {
      fmt::print("The word is '{}'\n", state->correctWord);
      return true;
    }
    if (isWord(input)) {
      state->makeGuess(input);
      return true;
    }
  } catch(const std::exception &ex) {
    std::cerr << "Error: " << ex.what() << '\n';
  }
  return false;
}

} // namespace

__attribute__((constructor)) void handleDlOpen() {
  std::puts("WordlePlugin (constructed)");
  fmt::print("- answer  list length: {}\n", answerWordCount);
  fmt::print("- allowed list length: {}\n", allowedWordCount);
}

__attribute__((destructor)) void handleDlClose() {
  std::puts("WordlePlugin (destructed)");
}

extern "C" {

auto getPluginFunctions() -> PluginFunctions {
  return PluginFunctions{
    .pluginInit = &pluginInit,
    .pluginPreReload = &pluginPreReload,
    .pluginPostReload = &pluginPostReload,
    .pluginDestroy = &pluginDestroy,
    .printHelp = &printHelp,
    .getCommands = nullptr,
    .tryHandleInput = &tryHandleInput,
  };
}

} // extern "C"
