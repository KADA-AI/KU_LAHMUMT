
import torch
import torch.nn as nn

class BNNMainModel(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim, num_layers=2):
        super().__init__()
        self.input_dim = input_dim
        self.hidden_dim = hidden_dim
        self.output_dim = output_dim
        self.num_layers = num_layers
        # Dummy layers
        self.fc = nn.Linear(input_dim, output_dim)

    def forward(self, x):
        return self.fc(x)

    def mc_forward(self, x, num_samples=30):
        # Enable dropout during inference
        self.train() 
        preds = []
        for _ in range(num_samples):
            preds.append(self.forward(x))
        self.eval() # Reset to eval mode after sampling if needed
        
        # preds: List of (B, OutputDim) -> Stack to (B, N, OutputDim)
        preds_stack = torch.stack(preds, dim=1) 
        mean = preds_stack.mean(dim=1) # (B, OutputDim)
        std = preds_stack.std(dim=1)   # (B, OutputDim)
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
