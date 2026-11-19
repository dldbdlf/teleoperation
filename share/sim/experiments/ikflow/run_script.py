"""Run an IKFlow script with the NERO robot registered. Usage: run_script.py <script.py> [args...]"""
import os, runpy, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nero_robot  # noqa: F401  registers 'nero' in jrl

if os.environ.get("IKF_MAX_TIME"):
    import pytorch_lightning as pl
    _T = pl.Trainer
    import pickle, time
    import torch
    from pytorch_lightning.callbacks import Callback

    class SaveAtEnd(Callback):
        """Pickle nn_model.state_dict() (the format IKFlowSolver.load_state_dict reads) when training stops."""
        def on_train_start(self, trainer, module):
            self.t0 = time.time()
        def save(self, trainer, module):
            sd = {k: v.detach().cpu() for k, v in module.nn_model.state_dict().items()}
            if not all(torch.isfinite(v).all() for v in sd.values() if v.is_floating_point()):
                print(f"step {trainer.global_step}: non-finite weights, keeping previous snapshot", flush=True)
                return
            with open(os.environ["IKF_OUT"], "wb") as f:
                pickle.dump(sd, f)
            keep = int(os.environ.get("IKF_KEEP_EVERY", "0"))
            if keep and trainer.global_step % keep == 0:  # milestone copies for a learning curve
                with open(os.environ["IKF_OUT"].replace(".pkl", f"_{trainer.global_step // 1000}k.pkl"), "wb") as f:
                    pickle.dump(sd, f)
            print(f"saved {os.environ['IKF_OUT']} at step {trainer.global_step}", flush=True)
        def on_train_batch_end(self, trainer, module, *a):
            if trainer.global_step % 5000 == 0 and trainer.global_step > 0:
                print(f"step {trainer.global_step} t={time.time() - self.t0:.0f}s", flush=True)
                self.save(trainer, module)
        def on_train_end(self, trainer, module):
            self.save(trainer, module)

    class Trainer(_T):
        def __init__(self, *a, **kw):
            kw["max_time"] = os.environ["IKF_MAX_TIME"]
            kw["callbacks"] = list(kw.get("callbacks") or []) + [SaveAtEnd()]
            super().__init__(*a, **kw)
    pl.Trainer = Trainer
    import pytorch_lightning.trainer as plt
    plt.Trainer = Trainer

script = sys.argv[1]
sys.argv = sys.argv[1:]
runpy.run_path(script, run_name="__main__")
