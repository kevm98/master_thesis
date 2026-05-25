from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Iterable

import numpy as np


@dataclass(frozen=True)
class SequenceSpec:
    """Specification for a fixed-size time window buffer."""

    window_size: int
    feature_dim: int


class SequenceBuffer:
    """Ring buffer that stores the last `window_size` feature vectors.

    Notes:
    - Stored as float32 numpy arrays.
    - `as_array(pad=True)` left-pads with zeros until the buffer is full.
    """

    def __init__(self, spec: SequenceSpec):
        if spec.window_size <= 0:
            raise ValueError("window_size must be > 0")
        if spec.feature_dim <= 0:
            raise ValueError("feature_dim must be > 0")

        self._spec = spec
        self._buf: Deque[np.ndarray] = deque(maxlen=spec.window_size)

    @property
    def spec(self) -> SequenceSpec:
        return self._spec

    def reset(self) -> None:
        self._buf.clear()

    def __len__(self) -> int:
        return len(self._buf)

    def append(self, x_t: Iterable[float] | np.ndarray) -> None:
        x = np.asarray(x_t, dtype=np.float32).reshape(-1)
        if x.shape[0] != self._spec.feature_dim:
            raise ValueError(f"Expected feature_dim={self._spec.feature_dim}, got {x.shape[0]} for x_t")
        self._buf.append(x)

    def as_array(self, *, pad: bool = True, pad_value: float = 0.0) -> np.ndarray:
        """Returns array of shape (window_size, feature_dim) if pad else (len, feature_dim)."""
        if not self._buf:
            if not pad:
                return np.zeros((0, self._spec.feature_dim), dtype=np.float32)
            return np.full((self._spec.window_size, self._spec.feature_dim), pad_value, dtype=np.float32)

        arr = np.stack(list(self._buf), axis=0).astype(np.float32, copy=False)
        if not pad:
            return arr

        if arr.shape[0] == self._spec.window_size:
            return arr

        pad_rows = self._spec.window_size - arr.shape[0]
        pad_block = np.full((pad_rows, self._spec.feature_dim), pad_value, dtype=np.float32)
        return np.concatenate([pad_block, arr], axis=0)
