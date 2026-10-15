class PolicyExporterHIM(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  actor : __torch__.torch.nn.modules.container.Sequential
  estimator : __torch__.torch.nn.modules.container.___torch_mangle_6.Sequential
  def forward(self: __torch__.legged_gym.utils.helpers.PolicyExporterHIM,
    obs_history: Tensor) -> Tensor:
    _0 = __torch__.torch.nn.functional.normalize
    estimator = self.estimator
    _1 = torch.slice((estimator).forward(obs_history, ))
    parts = torch.slice(_1, 1, 0, 19)
    vel = torch.slice(parts, -1, None, 3)
    z = torch.slice(parts, -1, 3)
    z0 = _0(z, 2., -1, 9.9999999999999998e-13, None, )
    actor = self.actor
    _2 = torch.slice(torch.slice(obs_history), 1, 0, 45)
    _3 = (actor).forward(torch.cat([_2, vel, z0], 1), )
    return _3
