// This example caused a failure
 
template<typename Dst, template <typename> class StorageBase, typename Src, typename Func>
 void call_assignment(NoAlias<Dst,StorageBase>& dst, const Src& src, const Func& func)
{
  call_assignment_no_alias(dst.expression(), src, func);
}

template<typename Dst, typename Src, typename Func>
 void call_assignment_no_alias(Dst& dst, const Src& src, const Func& func)
{

  enum {
    NeedToTranspose = ( (int(Dst::RowsAtCompileTime) == 1 && int(Src::ColsAtCompileTime) == 1)
                        || (int(Dst::ColsAtCompileTime) == 1 && int(Src::RowsAtCompileTime) == 1)
                      ) && int(Dst::SizeAtCompileTime) != 1
  };

}
