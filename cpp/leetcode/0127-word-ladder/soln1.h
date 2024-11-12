#ifndef SOLN_1_H
#define SOLN_1_H

#include <array>
#include <iostream>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

class Solution_1 {
    using WordSet = std::unordered_set<std::string_view>;

    struct State {
        std::string_view word;
        uint16_t cost;
        friend bool operator< (const State &a, const State &b) {
          return a.cost < b.cost;
        }
        friend std::ostream& operator<<(std::ostream &out, const State &a) {
            return out << "State(" << a.word << ", " << a.cost << ")";
        }
    };

    struct Trie {
        std::array<Trie*, 26> children;
        std::string_view word {}; // simultaneously marks end of word and has the full word

        ~Trie() noexcept {
            for (size_t i = 0; i < 26; ++i) {
                if (children[i]) {
                    delete children[i];
                }
            }
        }

        Trie*& child(char ch) { return children[ch - 'a']; }

        template <typename Iterator>
        Trie* add(std::string_view sv, Iterator start, Iterator stop) {
            auto current = this;
            for (; start != stop; ++start) {
                auto &my_child = current->child(*start);
                if (!my_child) {
                    my_child = new Trie();
                }
                current = my_child;
            }
            current->word = sv;
            return current;
        }

        template <typename Iterator>
        Trie *prefix(Iterator start, Iterator stop) {
            auto current = this;
            for (; start != stop && current; ++start) {
                current = current->child(*start);
            }
            return current;
        }

        template <typename Consumer>
        void for_each_word(Consumer consumer) {
            if (!word.empty()) {
                consumer(word);
            }
            for (auto my_child : children) {
                if (my_child) {
                    my_child->for_each_word(consumer);
                }
            }
        }

        template <typename Iterator>
        bool is_in(Iterator start, Iterator stop) {
            auto my_child = prefix(start, stop);
            return bool(my_child) && !my_child->word.empty();
        }

        template <typename Consumer>
        void for_each_char(Consumer consumer) {
            for (char ch = 'a'; ch <= 'z'; ++ch) {
                if (child(ch)) {
                    consumer(ch);
                }
            }
        }
    }; // end of struct Trie

public:
    int ladderLength(std::string beginWord, std::string endWord,
                     std::vector<std::string>& wordList)
    {
        // This is a graph search problem over a large implicit graph
        // This is where memory complexity can really get into the picture
        // Methods that will work
        // - BFS (queue)
        // - Iterative Deepening (stack with max depth)
        // - Uniform Cost Search (Dijkstra's) (priority queue with cost)
        // - A* Search (priority queue with cost + heuristic)
        //   - Heuristics:
        //     - Edit distance (# diff characters)
        //       - Admissible (Same as cost if all words are possible)
        //       - Consistent (Satisfies triangle inequality)
        // Things to think about
        // - Efficient way to get all neighbors
        //   - Maybe a Trie?  A prefix trie and a postfix trie?
        //   - Maybe an explicit adjacency list, pregenerated?
        //   - Try every possible child and check if it is in the set? (seems bad)

        wordList.emplace_back(beginWord);
        Trie *prefix_trie = new Trie();
        Trie *postfix_trie = new Trie();
        for (auto &word : wordList) {
            prefix_trie->add(word, word.begin(), word.end());
            postfix_trie->add(word, word.rbegin(), word.rend());
        }

        // Frontier:
        // - PushFunc
        // - PopFunc
        // - IsFrontierEmptyFunc
        // This particular one does A* search
        auto heuristic = [&endWord](const State &state) {
            decltype(state.cost) dist = 0;
            for (size_t i = 0; i < endWord.size(); ++i) {
                dist += (endWord[i] != state.word[i]);
            }
            return dist;
        };
        auto comp = [&heuristic](const State &a, const State &b) {
            return (a.cost + heuristic(a)) > (b.cost + heuristic(b));
        };
        std::priority_queue<State, std::vector<State>, decltype(comp)> frontier(comp);
        auto push = [&frontier](const State &state) {
            //std::cout << "push: " << state << "\n";
            frontier.push(state);
        };
        auto pop = [&frontier]() {
            auto state = frontier.top();
            frontier.pop();
            //std::cout << "pop: " << state << "\n";
            return state;
        };
        auto is_frontier_empty = [&frontier]() { return frontier.empty(); };

        // Visited:
        // - IsVisitedFunc
        // - VisitFunc
        std::unordered_set<std::string_view> visited;
        auto is_visited = [&visited](const State &state) {
            bool ans = (1 == visited.count(state.word));
            //std::cout << "is_visited: " << state << " := " << ans << "\n";
            return ans;
        };
        auto visit = [&visited](const State &state) {
            //std::cout << "visit: " << state << "\n";
            visited.emplace(state.word);
        };

        // Goal:
        // - IsGoalFunc
        auto is_goal = [&endWord](const State &state) {
            bool ans = (endWord == state.word);
            //std::cout << "is_goal: " << state << " := " << ans << "\n";
            return ans;
        };

        // Neighbors:
        // - ForEachNeighborFunc
        //   Needs to enumerate all neighbors and call the consumer function
        auto for_each_neighbor =
            [&prefix_trie, &postfix_trie](const State &state, auto consumer) {
                // go through each of the positions where a letter can be different
                for (size_t i = 0; i < state.word.size(); ++i) {
                    auto pre_subtrie = prefix_trie->prefix(state.word.begin(),
                                                           state.word.begin()+i);
                    auto postfix = state.word.substr(i+1);
                    for_each_common_char(
                        pre_subtrie,
                        postfix_trie->prefix(postfix.rbegin(), postfix.rend()),
                        [&state, &consumer, i, pre_subtrie, postfix](char ch) {
                            if (ch == state.word[i]) { return; }
                            State neighbor;
                            neighbor.cost = state.cost + 1;
                            auto found = pre_subtrie->child(ch);
                            found = found->prefix(postfix.begin(), postfix.end());
                            if (!found) {
                                //std::cout << "not found word: " << state << " for " << i << "th char = " << ch << " (postfix: " << postfix << ")" << std::endl;
                                return; // filters out false join positives
                            }
                            neighbor.word = found->word;
                            consumer(neighbor);
                        }
                    );
                }
            };

        State start;
        start.word = beginWord;
        start.cost = 1;
        push(start);
        auto final_state = graph_search<State>(
            push, pop, is_frontier_empty,
            is_visited, visit,
            is_goal,
            for_each_neighbor
        );
        std::cout << "num visited: " << visited.size() << "\n";
        return final_state.cost;
    }


private:
    // generator function so that we can have it const after creation
    WordSet make_wordset(const std::vector<std::string> &wordList,
                         const std::string beginWord,
                         const std::string endWord) const
    {
        WordSet words(wordList.begin(), wordList.end(),
                      2*wordList.size());
        words.emplace(beginWord);
        words.emplace(endWord);
        return words;
    }

    template <typename Consumer>
    static
    void for_each_common_char(Trie *a, Trie *b, Consumer consumer) {
        for (char ch = 'a'; ch <= 'z'; ++ch) {
            if (a->child(ch) && b->child(ch)) {
                consumer(ch);
            }
        }
    }

    template <
        typename StateType,
        typename PushFunc,
        typename PopFunc,
        typename IsFrontierEmptyFunc,
        typename IsVisitedFunc,
        typename VisitFunc,
        typename IsGoalFunc,
        typename ForEachNeighborFunc
    >
    static
    StateType graph_search(
        PushFunc push,
        PopFunc pop,
        IsFrontierEmptyFunc is_frontier_empty,
        IsVisitedFunc is_visited,
        VisitFunc visit,
        IsGoalFunc is_goal,
        ForEachNeighborFunc for_each_neighbor
    )
    {
        while (!is_frontier_empty()) {
            StateType state = pop();
            if (is_goal(state)) { return state; }
            if (is_visited(state)) { continue; }
            visit(state);
            for_each_neighbor(state, [&](const State &neighbor) {
                if (is_visited(neighbor)) { return; }
                push(neighbor);
            });
        }
        return {};
    }
};

#endif // SOLN_1_H
