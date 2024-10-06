#include <array>
#include <functional>
#include <iostream>
#include <string_view>
#include <utility>

#include <cstdint>

namespace tt {

///
/// NthElement - Helper for accessing an element at a given index within a
/// variadic set of types.
///
template <uint64_t Index, typename... Ts> struct NthElement;

template <uint64_t Index, typename Head, typename... Ts>
struct NthElement<Index, Head, Ts...> {
  using type = typename NthElement<Index - 1, Ts...>::type;
};

template <typename Head, typename... Ts> struct NthElement<0, Head, Ts...> {
  using type = Head;
};

///
/// PPack - Helper for querying type lists.
///
template <typename... Ts> struct PPack {
  static constexpr uint64_t count = sizeof...(Ts);

  template <uint64_t Index>
  using element = typename NthElement<Index, Ts...>::type;
};

///
/// Helper for accessing the type at a given index in a variadic parameter list
/// or PPack.
///
template <uint64_t I, typename PPackType>
using ElementT = typename PPackType::template element<I>;

///
/// AsPPack - Helper for converting types to a PPack.
///
template <typename T> struct AsPPack {
  using type = PPack<T>;
};

template <typename... Ts> struct AsPPack<std::tuple<Ts...>> {
  using type = PPack<Ts...>;
};

template <> struct AsPPack<void> {
  using type = PPack<>;
};

template <typename T> using as_ppack_t = typename AsPPack<T>::type;

///
/// ArgumentPack - Helper for deducing argument types to a function or
/// struct/class call operator.
///
template <typename T, typename Enable = void> struct ArgumentPack;

template <typename T> using ArgumentPackT = typename ArgumentPack<T>::type;

template <typename Ret, typename... Args>
struct ArgumentPack<Ret (*)(Args...)> {
  using type = PPack<Args...>;
};

template <typename Ret, typename Class, typename... Args>
struct ArgumentPack<Ret (Class::*)(Args...) const> {
  using type = PPack<Args...>;
};

template <typename Ret, typename Class, typename... Args>
struct ArgumentPack<Ret (Class::*)(Args...)> {
  using type = PPack<Args...>;
};

template <typename Callable>
struct ArgumentPack<Callable, std::void_t<decltype(&Callable::operator())>> {
  using type = ArgumentPackT<decltype(&Callable::operator())>;
};

template <typename T, typename Enable = void> struct ReturnType;

template <typename T> using ReturnT = typename ReturnType<T>::type;

template <typename Ret, typename... Args> struct ReturnType<Ret (*)(Args...)> {
  using type = Ret;
};

template <typename Ret, typename Class, typename... Args>
struct ReturnType<Ret (Class::*)(Args...) const> {
  using type = Ret;
};

template <typename Ret, typename Class, typename... Args>
struct ReturnType<Ret (Class::*)(Args...)> {
  using type = Ret;
};

template <typename Callable>
struct ReturnType<Callable, std::void_t<decltype(&Callable::operator())>> {
  using type = ReturnT<std::remove_const_t<decltype(&Callable::operator())>>;
};

template <typename ReturnType, typename ArgPack> struct FunctionSignatureImpl;

template <typename ReturnType, template <typename...> typename Pack,
          typename... Args>
struct FunctionSignatureImpl<ReturnType, Pack<Args...>> {
  using type = ReturnType(Args...);
};

template <typename Callable>
using FunctionSignature =
    FunctionSignatureImpl<ReturnT<Callable>, ArgumentPackT<Callable>>::type;

template <typename Callable>
using FunctionType = std::function<FunctionSignature<Callable>>;

template <typename Callable, typename ReturnType, typename ArgPack>
struct IsInvocableByPackImpl;

template <typename Callable, typename ReturnType, typename... Args>
struct IsInvocableByPackImpl<Callable, ReturnType, PPack<Args...>>
    : std::is_invocable_r<ReturnType, Callable, Args...> {};

template <typename Callable>
struct IsSingleCallable : IsInvocableByPackImpl<Callable, ReturnT<Callable>,
                                                ArgumentPackT<Callable>> {};

template <typename Callable>
inline constexpr auto IsSingleCallableV = IsSingleCallable<Callable>::value;

template <typename Callable>
concept SingleCallable = IsSingleCallable<Callable>::value;

template <typename Callable, typename ReturnType, typename... Args>
concept invocable_r = std::is_invocable_r<ReturnType, Callable, Args...>::value;

} // namespace tt

template <class ReturnType, class... ArgTypes> class CounterPrintDecorator {
public:
  CounterPrintDecorator(std::function<ReturnType(ArgTypes...)> callable)
      : callable_{std::move(callable)} {}

  auto operator()(ArgTypes &&...args) -> ReturnType {
    std::cout << "V1 Counter: " << ++counter_ << "\n";
    return std::invoke(callable_, std::forward<ArgTypes>(args)...);
  }

private:
  std::function<ReturnType(ArgTypes...)> callable_;
  std::size_t counter_{0};
};

template <class ReturnType, class... Args>
auto makeCounterPrintDecorator_v2(std::function<ReturnType(Args...)> callable)
    -> decltype(auto) {
  auto decorated = [capturedCallable = std::move(callable),
                    counter =
                        std::size_t{0}](Args... args) mutable -> ReturnType {
    std::cout << "V2 Counter: " << ++counter << ": ";
    return std::invoke(capturedCallable, std::forward<Args>(args)...);
  };
  using DecoratedType = decltype(decorated);
  static_assert(std::is_same_v<tt::ReturnT<DecoratedType>, ReturnType>);
  static_assert(std::is_invocable_r_v<ReturnType, DecoratedType, Args...>);
  static_assert(
      std::is_same_v<tt::ArgumentPackT<DecoratedType>, tt::PPack<Args...>>);
  return decorated;
}

template <class Callable>
auto makeCounterPrintDecorator_v2(Callable callable) -> decltype(auto) {
  return makeCounterPrintDecorator_v2(
      tt::FunctionType<Callable>{std::move(callable)});
}

/// V3: Makes a non-templated lambda, but directly matches the interface of the
/// input callable. Requires an impl class below that can inject the input
/// arguments as a non-packed variadic set.
template <class... Args>
auto makeCounterPrintDecorator_v3(std::invocable<Args...> auto callable)
    -> decltype(auto) {
  using Callable = decltype(callable);
  using Ret = tt::ReturnT<Callable>;
  auto decorated = [capturedCallable = std::move(callable),
                    counter = std::size_t{0}](Args... args) mutable -> Ret {
    std::cout << "V3 Counter: " << ++counter << ": ";
    return std::invoke(capturedCallable, std::forward<Args>(args)...);
  };
  using DecoratedType = decltype(decorated);
  static_assert(std::is_same_v<tt::ReturnT<DecoratedType>, Ret>);
  static_assert(std::is_invocable_r_v<Ret, DecoratedType, Args...>);
  static_assert(
      std::is_same_v<tt::ArgumentPackT<DecoratedType>, tt::PPack<Args...>>);
  return decorated;
}

template <class Callable, class ArgPack>
  requires std::is_same_v<tt::ArgumentPackT<Callable>, ArgPack>
struct MakeCounterPrintDecorator_v3_impl;

template <class Callable, class... Args>
struct MakeCounterPrintDecorator_v3_impl<Callable, tt::PPack<Args...>> {
  static auto make(Callable callable) -> decltype(auto) {
    return makeCounterPrintDecorator_v3<Args...>(std::move(callable));
  }
};

template <class Callable>
auto makeCounterPrintDecorator_v3(Callable callable) -> decltype(auto) {
  return MakeCounterPrintDecorator_v3_impl<
      Callable, tt::ArgumentPackT<Callable>>::make(callable);
}

/// V4: Makes a templated lambda then places it in a std::function to make it
/// work with ReturnT and ArgumentPackT
template <class Callable>
auto makeCounterPrintDecorator_v4(Callable callable) -> decltype(auto) {
  auto decorated =
      tt::FunctionType<Callable>{[capturedCallable = std::move(callable),
                                  counter = std::size_t{0}]<typename... Args>(
                                     Args &&...args) mutable -> decltype(auto) {
        std::cout << "V4 Counter: " << ++counter << ": ";
        return std::invoke(capturedCallable, std::forward<Args>(args)...);
      }};
  using DecoratedType = decltype(decorated);
  static_assert(
      std::is_same_v<tt::ReturnT<DecoratedType>, tt::ReturnT<Callable>>);
  static_assert(std::is_same_v<tt::ArgumentPackT<DecoratedType>,
                               tt::ArgumentPackT<Callable>>);
  return decorated;
}

template <typename Pack> struct callWithTemplateArgs_impl;

template <template <typename...> typename PackContainer, typename... Args>
struct callWithTemplateArgs_impl<PackContainer<Args...>> {
  template <typename... TargetArgs>
  static auto call(auto callable, TargetArgs &&...targetArgs)
      -> decltype(auto) {
    return callable.template operator()<Args...>(
        std::forward<TargetArgs>(targetArgs)...);
  }
};

template <typename Pack, typename... TargetArgs>
auto callWithTemplateArgs(auto callable, TargetArgs &&...targetArgs)
    -> decltype(auto) {
  return callWithTemplateArgs_impl<Pack>::call(
      std::move(callable), std::forward<TargetArgs>(targetArgs)...);
}

/// V5: Makes a non-std::function return type using multiple lambdas and
/// callWithTemplateArgs()
template <class Callable>
auto makeCounterPrintDecorator_v5(Callable callable) -> decltype(auto) {
  const auto makeDecorator = [&callable]<typename... Args>() {
    return [capturedCallable = std::move(callable),
            counter = std::size_t{0}](Args... args) mutable -> decltype(auto) {
      std::cout << "V5 Counter: " << ++counter << ": ";
      return std::invoke(capturedCallable, std::forward<Args>(args)...);
    };
  };
  auto decorated =
      callWithTemplateArgs<tt::ArgumentPackT<Callable>>(makeDecorator);

  using DecoratedType = decltype(decorated);
  static_assert(
      std::is_same_v<tt::ReturnT<DecoratedType>, tt::ReturnT<Callable>>);
  static_assert(std::is_same_v<tt::ArgumentPackT<DecoratedType>,
                               tt::ArgumentPackT<Callable>>);

  return decorated;
}

int main() {
  const auto printName = [](std::string_view name) -> std::ostream & {
    return std::cout << "My name is " << name << ".\n";
  };
  // CounterPrintDecorator decoratedV1{printName};
  auto decoratedV2 = makeCounterPrintDecorator_v2(printName);
  auto decoratedV3 = makeCounterPrintDecorator_v3(printName);
  auto decoratedV4 = makeCounterPrintDecorator_v4(printName);
  auto decoratedV5 = makeCounterPrintDecorator_v5(printName);

  using namespace std::literals::string_view_literals;

  const std::array names{"A"sv, "B"sv, "C"sv, "D"sv};
  for (const auto &name : names) {
    printName(name);
  }
  for (const auto &name : names) {
    decoratedV2(name);
  }
  for (const auto &name : names) {
    decoratedV3(name);
  }
  for (const auto &name : names) {
    decoratedV4(name);
  }
  for (const auto &name : names) {
    decoratedV5(name);
  }

  return 0;
}
