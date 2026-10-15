class Sequential(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : NoneType
  __annotations__["0"] = __torch__.torch.nn.modules.linear.Linear
  __annotations__["1"] = __torch__.torch.nn.modules.activation.ELU
  __annotations__["2"] = __torch__.torch.nn.modules.linear.___torch_mangle_0.Linear
  __annotations__["3"] = __torch__.torch.nn.modules.activation.ELU
  __annotations__["4"] = __torch__.torch.nn.modules.linear.___torch_mangle_1.Linear
  __annotations__["5"] = __torch__.torch.nn.modules.activation.ELU
  __annotations__["6"] = __torch__.torch.nn.modules.linear.___torch_mangle_2.Linear
  def forward(self: __torch__.torch.nn.modules.container.Sequential,
    input: Tensor) -> Tensor:
    _0 = getattr(self, "0")
    _1 = getattr(self, "1")
    _2 = getattr(self, "2")
    _3 = getattr(self, "3")
    _4 = getattr(self, "4")
    _5 = getattr(self, "5")
    _6 = getattr(self, "6")
    input0 = (_0).forward(input, )
    input1 = (_1).forward(input0, )
    input2 = (_2).forward(input1, )
    input3 = (_3).forward(input2, )
    input4 = (_4).forward(input3, )
    input5 = (_5).forward(input4, )
    return (_6).forward(input5, )
  def __len__(self: __torch__.torch.nn.modules.container.Sequential) -> int:
    return 7
