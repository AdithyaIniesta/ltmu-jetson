#!/usr/bin/env python3
"""Export the ResNet-18 appearance embedder to ONNX.

Same model as the validated Python prototype (ltmu-tracker/ltmu/embedder.py):
torchvision ResNet-18, ImageNet weights, truncated at avgpool, L2-normalized
output. The C++ tracker (src/tracker/embedder_onnx.cpp) loads the exported
file via ONNX Runtime.

Usage:
    python3 scripts/export_onnx.py --out models/resnet18_embedder.onnx
"""
import argparse

import torch
import torch.nn as nn
from torchvision.models import ResNet18_Weights, resnet18


class L2NormEmbedder(nn.Module):
    def __init__(self):
        super().__init__()
        net = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
        self.backbone = nn.Sequential(*list(net.children())[:-1])  # drop fc

    def forward(self, x):
        feats = self.backbone(x).flatten(1)
        return torch.nn.functional.normalize(feats, dim=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="models/resnet18_embedder.onnx")
    ap.add_argument("--img-size", type=int, default=128)
    args = ap.parse_args()

    model = L2NormEmbedder().eval()
    dummy = torch.randn(1, 3, args.img_size, args.img_size)

    torch.onnx.export(
        model,
        dummy,
        args.out,
        input_names=["input"],
        output_names=["embedding"],
        dynamic_axes={"input": {0: "batch"}, "embedding": {0: "batch"}},
        opset_version=17,
    )
    print(f"exported {args.out}")


if __name__ == "__main__":
    main()
