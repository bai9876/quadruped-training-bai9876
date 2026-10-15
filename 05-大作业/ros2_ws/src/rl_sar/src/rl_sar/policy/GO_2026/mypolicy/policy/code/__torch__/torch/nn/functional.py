def normalize(input: Tensor,
    p: float=2.,
    dim: int=1,
    eps: float=9.9999999999999998e-13,
    out: Optional[Tensor]=None) -> Tensor:
  if torch.__is__(out, None):
    _1 = torch.clamp_min(torch.norm(input, p, [dim], True), eps)
    denom = torch.expand_as(_1, input)
    _0 = torch.div(input, denom)
  else:
    out0 = unchecked_cast(Tensor, out)
    _2 = torch.clamp_min_(torch.norm(input, p, [dim], True), eps)
    denom0 = torch.expand_as(_2, input)
    _0 = torch.div(input, denom0, out=out0)
  return _0
def elu(input: Tensor,
    alpha: float=1.,
    inplace: bool=False) -> Tensor:
  if inplace:
    result = torch.elu_(input, alpha)
  else:
    result = torch.elu(input, alpha)
  return result
