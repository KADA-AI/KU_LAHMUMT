
import torch
import torch.nn as nn


class BNNMainModel(nn.Module):
    def __init__(
        self,
        input_dim,
        hidden_dim,
        output_dim=6,
        dropout_prob=0.3,
        num_layers=2,
    ):
        super().__init__()
        self.dropout_prob = dropout_prob

        layers = []
        layers.append(nn.Linear(input_dim, hidden_dim))
        layers.append(nn.ReLU())
        layers.append(nn.Dropout(p=dropout_prob))

        for _ in range(num_layers - 1):
            layers.append(nn.Linear(hidden_dim, hidden_dim))
            layers.append(nn.ReLU())
            layers.append(nn.Dropout(p=dropout_prob))

        layers.append(nn.Linear(hidden_dim, output_dim))
        layers.append(nn.Sigmoid())

        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)

    def mc_forward(self, x, num_samples=30):
        self.train()
        preds = []
        for _ in range(num_samples):
            preds.append(self.forward(x))
        self.eval()

        preds_stack = torch.stack(preds, dim=1)
        mean = preds_stack.mean(dim=1)
        std = preds_stack.std(dim=1)
        return mean, std


class LSTMAutoencoder(nn.Module):
    class _Encoder(nn.Module):
        def __init__(self, input_dim, hidden_dim, num_layers):
            super().__init__()
            self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)

    class _Decoder(nn.Module):
        def __init__(self, input_dim, hidden_dim, num_layers):
            super().__init__()
            self.lstm = nn.LSTM(hidden_dim, hidden_dim, num_layers, batch_first=True)
            self.fc_out = nn.Linear(hidden_dim, input_dim)

    def __init__(self, input_dim, hidden_dim, num_layers=1):
        super().__init__()
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers
        self.encoder = self._Encoder(input_dim, hidden_dim, num_layers)
        self.decoder = self._Decoder(input_dim, hidden_dim, num_layers)

    def encode(self, x, lengths=None):
        if lengths is not None:
            packed = nn.utils.rnn.pack_padded_sequence(
                x, lengths.cpu(), batch_first=True, enforce_sorted=False
            )
            _, (h_n, _c_n) = self.encoder.lstm(packed)
        else:
            _, (h_n, _c_n) = self.encoder.lstm(x)
        return h_n[-1]

    def forward(self, x, lengths=None):
        # Returns (reconstruction, embedding)
        emb = self.encode(x, lengths=lengths)
        seq_len = x.size(1)
        dec_in = emb.unsqueeze(1).repeat(1, seq_len, 1)
        dec_out, _ = self.decoder.lstm(dec_in)
        recon = self.decoder.fc_out(dec_out)
        return recon, emb
