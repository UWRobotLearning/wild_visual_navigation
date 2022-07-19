import torch

activation = {"sigmoid": torch.nn.Sigmoid(), "relu": torch.nn.ReLU()}


class SimpleMLP(torch.nn.Module):
    def __init__(self, input_size=64, hidden_sizes=[255], final_activation="sigmoid"):
        super(SimpleMLP, self).__init__()

        layers = []
        for hs in hidden_sizes[:1]:
            layers.append(torch.nn.Linear(input_size, hs))
            layers.append(torch.nn.ReLU())
            input_size = hs
        layers.append(torch.nn.Linear(input_size, hidden_sizes[-1]))
        layers.append(activation[final_activation])

        self.layers = torch.nn.Sequential(layers)

    def forward(self, x):
        return self.layers(x)