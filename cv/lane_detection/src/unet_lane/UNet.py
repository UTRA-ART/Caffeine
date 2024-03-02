from operator import indexOf
from numpy import ndim
import torch.nn as nn
import torch


class ConvStage(nn.Module):
    def __init__(self, in_channels, out_channels) -> None:
        super(ConvStage, self).__init__()
        self.conv = nn.Sequential(
            nn.Conv2d(in_channels, out_channels, 3, 1, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, 3, 1, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.conv(x)


class UNet(nn.Module):
    def __init__(self, in_channels=4, out_channels=1, features=[16, 32, 64, 128]):
        super(UNet, self).__init__()

        self.encoder = nn.ModuleList()
        self.decoder = nn.ModuleList()
        self.pool = nn.MaxPool2d(2, 2)

        # Setting up the architecture for the encoder
        for feature in features:
            double_conv = ConvStage(
                in_channels=in_channels, out_channels=feature)
            self.encoder.append(double_conv)
            in_channels = feature

        self.bottleneck = ConvStage(
            in_channels=features[-1], out_channels=features[-1] * 2
        )

        # Setting up the architecrue for the decoder
        for feature in reversed(features):
            up_conv = nn.ConvTranspose2d(
                in_channels=feature * 2, out_channels=feature, kernel_size=2, stride=2
            )
            self.decoder.append(up_conv)
            double_conv = ConvStage(
                in_channels=feature * 2, out_channels=feature)
            self.decoder.append(double_conv)

        self.segmentation = nn.Conv2d(
            in_channels=features[0], out_channels=out_channels, kernel_size=1, stride=1
        )

    def forward(self, x):
        # Make sure that the inputted size is compatible
        assert x.shape[2] % 16 == 0 and x.shape[3] % 16 == 0

        copies = []

        # Forward pass through the encoder
        for i, down in enumerate(self.encoder):
            x = down(x)
            # Store a copy
            copies.append(x)
            x = self.pool(x)

        # The bottleneck
        x = self.bottleneck(x)

        # Reverse the coppies
        copies = copies[::-1]

        # Forward pass through the decoder
        for j, up in enumerate(self.decoder):
            if j % 2 == 0:
                x = up(x)
                x = torch.cat((copies[j // 2], x), axis=1)
            else:
                x = up(x)

        return self.segmentation(x)


def test():
    x = torch.rand((3, 2, 128, 128))
    model = UNet(in_channels=2, out_channels=1)
    pred = model(x)
    print(x.shape, pred.shape)


if __name__ == "__main__":
    weight_pth = r"C:\Users\ammar\Documents\CodingProjects\ART\CV-Pipeline\src\lane_detection\UNet-LaneDetection\runs\1668151763.9445446\1668151763.9445446unet_gray_model_batch64_sheduled_lr0.1_epochs15.pt"

    model = UNet()
    model.load_state_dict(torch.load(
        weight_pth, map_location=torch.device("cpu")))
    # model.to("cpu")
    model.eval()
    dummy_input = torch.ones(1, 4, 160, 256, device="cpu")

    out = model(dummy_input)
    print(out.shape)
    print(out)

    torch.onnx.export(model,
                     dummy_input,
                     "unet_v1.onnx",
                     opset_version=11,
                     do_constant_folding=True,
                     input_names=["Inputs"],
                     output_names=["Outputs"])
    
    print("Onnx conversion complete")
