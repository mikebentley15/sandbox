#include <iostream>
#include <memory>

struct HtmlElement {
  std::string data;

  explicit HtmlElement(std::string &&_data) : data(_data) {}
};

struct Node {
  Node* parent;
  std::vector<Node> children;
  std::unique_ptr<HtmlElement> html;

  Node(std::string _data, Node *_parent = nullptr)
    : html(new HtmlElement(std::move(_data))), parent(_parent)
  { }

  // used when children resizes and moves the elements over
  Node(Node &&other) // move
    : parent(other.parent)
    , children(std::move(other.children))
    , html(std::move(other.html))
  {
    // update children pointers
    for (auto &child : children) { child.parent = this; }
  }

  template <typename Visitor>
  void visit_preorder(Visitor visitor) {
    int depth = 0;
    Node* current = this;

    // visit order: current, then children left to right (NLR)

    // go down and visit as you go starting with this one
    // this is the only piece that does actual visiting.
    auto go_down = [&depth, &current]() {
      visitor(current, depth);
      while (!current->children.empty()) {
        current = &current->children[0];
        ++depth;
        visitor(current, depth);
      }
    };

    // goes up until there is a next child from one of the parents
    // sets current to that next child and exits, ready to go down.
    // sets current to nullptr when no more nodes to visit
    auto go_up = [&depth, &current]() {
      while (current->parent != nullptr) {
        auto child = current;
        current = current->parent;
        --depth;
        if (child != &current->children.back()) {
          current = child + 1; // pointer arithmetic.  std::vector is contiguous
          ++depth;
          break;
        }
      }

      // we either exited the while loop because we have a new subtree to go
      // down or because current->parent is null
      if (current->parent == nullptr) {
        current = nullptr; // signal to be done
      }
    };

    // the basic algorithm.
    while (current != nullptr) {
      go_down(); // go down until you can't anymore
      go_up();   // go up until you find the next subtree
    }
  }
};

int main(int arg_count, char *arg_list[]) {
  // make the tree
  std::string val = "A";
  Node root(val); ++val[0];

  // Tree I'll make:
  //          __________________A_________
  //         |                  |         |
  //    _____B_____        _____O__     __W__
  //   |     |     |      |     |  |   |  |  |
  //  _C_   _G_   _K_    _P_    T  V   X  Y  Z
  // | | | | | | | | |  | | |   |
  // D E F H I J L M N  Q R S   U
  //

  auto make_child = [&val](Node *parent) {
    parent->children.emplace_back(val, parent);
    ++val[0];
    return &(parent->children.back());
  };

  auto a = &root;
  auto b = make_child(a);
  auto c = make_child(b);
  make_child(c);
  make_child(c);
  make_child(c);
  auto g = make_child(b);
  make_child(g);
  make_child(g);
  make_child(g);
  auto k = make_child(b);
  make_child(k);
  make_child(k);
  make_child(k);
  auto o = make_child(a);
  auto p = make_child(o);
  make_child(p);
  make_child(p);
  make_child(p);
  auto t = make_child(o);
  make_child(t);
  make_child(o);
  auto w = make_child(a);
  make_child(w);
  make_child(w);
  make_child(w);

  // visit the tree, and print node information
  root.visit_preorder([](auto *node, int depth) {
    std::cout << "Node(html=" << node->html->data
              << ", depth=" << depth
              << ", addr=" << node
              << ", parent=" << node->parent
              << ")\n";
  });
  std::cout << std::endl;

  // visit the tree, should print the uppercase alphabet
  root.visit_preorder([](auto *node, int depth) { std::cout << node->html->data; });
  std::cout << "\n" << std::endl;

  // visit the tree, should print the alphabet in the tree structure
  root.visit_preorder([](auto *node, int depth) {
    std::cout << "(" << depth << ") ";
    for (int i = 0; i < depth; ++i) {
      std::cout << "  ";
    }
    std::cout << node->html->data << std::endl;
  });
  std::cout << std::endl;

  return 0;
}
