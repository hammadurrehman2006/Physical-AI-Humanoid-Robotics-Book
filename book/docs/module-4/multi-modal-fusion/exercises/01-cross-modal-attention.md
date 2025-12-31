# Exercise 5.1: Cross-Modal Attention and Fusion

## Objective
Implement cross-modal attention mechanisms and fusion strategies that combine information from vision, language, and action systems for the Vision-Language-Action system.

## Prerequisites
- Python 3.10+
- PyTorch and related libraries
- Understanding of neural networks and attention mechanisms
- Completion of previous phases (Vision, Language, Action)
- Basic knowledge of multi-modal learning concepts

## Exercise Steps

### Step 1: Set Up Multi-Modal Fusion Environment
Create a new file `cross_modal_fusion.py`:

```python
#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Dict, List, Tuple, Any, Optional
import math

class CrossModalAttention(nn.Module):
    """Cross-modal attention mechanism to combine different modalities"""

    def __init__(self, vision_dim: int, language_dim: int, hidden_dim: int = 512, num_heads: int = 8):
        """
        Initialize cross-modal attention module

        Args:
            vision_dim: Dimension of vision features
            language_dim: Dimension of language features
            hidden_dim: Hidden dimension for attention computation
            num_heads: Number of attention heads
        """
        super().__init__()
        self.vision_dim = vision_dim
        self.language_dim = language_dim
        self.hidden_dim = hidden_dim
        self.num_heads = num_heads
        self.head_dim = hidden_dim // num_heads

        assert self.head_dim * num_heads == hidden_dim, "hidden_dim must be divisible by num_heads"

        # Linear projections for query, key, value
        self.vision_query = nn.Linear(vision_dim, hidden_dim)
        self.vision_key = nn.Linear(vision_dim, hidden_dim)
        self.vision_value = nn.Linear(vision_dim, hidden_dim)

        self.language_query = nn.Linear(language_dim, hidden_dim)
        self.language_key = nn.Linear(language_dim, hidden_dim)
        self.language_value = nn.Linear(language_dim, hidden_dim)

        # Output projections
        self.vision_output = nn.Linear(hidden_dim, hidden_dim)
        self.language_output = nn.Linear(hidden_dim, hidden_dim)

        # Layer normalization
        self.vision_norm = nn.LayerNorm(hidden_dim)
        self.language_norm = nn.LayerNorm(hidden_dim)

        # Dropout
        self.dropout = nn.Dropout(0.1)

    def forward(self, vision_features: torch.Tensor,
                language_features: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Compute cross-modal attention between vision and language features

        Args:
            vision_features: [batch_size, num_regions, vision_dim]
            language_features: [batch_size, seq_len, language_dim]

        Returns:
            attended_vision: [batch_size, num_regions, hidden_dim]
            attended_language: [batch_size, seq_len, hidden_dim]
        """
        batch_size, num_regions, _ = vision_features.shape
        _, seq_len, _ = language_features.shape

        # Project features to query, key, value
        v_queries = self.vision_query(vision_features).view(batch_size, num_regions, self.num_heads, self.head_dim)
        v_keys = self.vision_key(vision_features).view(batch_size, num_regions, self.num_heads, self.head_dim)
        v_values = self.vision_value(vision_features).view(batch_size, num_regions, self.num_heads, self.head_dim)

        l_queries = self.language_query(language_features).view(batch_size, seq_len, self.num_heads, self.head_dim)
        l_keys = self.language_key(language_features).view(batch_size, seq_len, self.num_heads, self.head_dim)
        l_values = self.language_value(language_features).view(batch_size, seq_len, self.num_heads, self.head_dim)

        # Transpose for attention computation: [batch, num_heads, seq_len, head_dim]
        v_queries = v_queries.transpose(1, 2)
        v_keys = v_keys.transpose(1, 2)
        v_values = v_values.transpose(1, 2)

        l_queries = l_queries.transpose(1, 2)
        l_keys = l_keys.transpose(1, 2)
        l_values = l_values.transpose(1, 2)

        # Compute vision attending to language
        vision_to_lang_attn = torch.matmul(v_queries, l_keys.transpose(-2, -1)) / math.sqrt(self.head_dim)
        vision_to_lang_attn = F.softmax(vision_to_lang_attn, dim=-1)
        vision_to_lang_attn = self.dropout(vision_to_lang_attn)
        vision_attended = torch.matmul(vision_to_lang_attn, l_values)

        # Compute language attending to vision
        lang_to_vision_attn = torch.matmul(l_queries, v_keys.transpose(-2, -1)) / math.sqrt(self.head_dim)
        lang_to_vision_attn = F.softmax(lang_to_vision_attn, dim=-1)
        lang_to_vision_attn = self.dropout(lang_to_vision_attn)
        lang_attended = torch.matmul(lang_to_vision_attn, v_values)

        # Reshape back to [batch, seq_len, hidden_dim]
        vision_attended = vision_attended.transpose(1, 2).contiguous().view(batch_size, num_regions, self.hidden_dim)
        lang_attended = lang_attended.transpose(1, 2).contiguous().view(batch_size, seq_len, self.hidden_dim)

        # Apply output projections
        vision_output = self.vision_output(vision_attended)
        lang_output = self.language_output(lang_attended)

        # Apply layer normalization
        attended_vision = self.vision_norm(vision_features + vision_output)
        attended_language = self.language_norm(language_features + lang_output)

        return attended_vision, attended_language

class SimpleCrossModalAttention(nn.Module):
    """Simplified cross-modal attention for easier understanding"""

    def __init__(self, vision_dim: int, language_dim: int, hidden_dim: int = 256):
        super().__init__()
        self.vision_proj = nn.Linear(vision_dim, hidden_dim)
        self.language_proj = nn.Linear(language_dim, hidden_dim)
        self.attention_weights = nn.Linear(hidden_dim * 2, 1)
        self.output_proj = nn.Linear(hidden_dim * 2, hidden_dim)

    def forward(self, vision_features: torch.Tensor,
                language_features: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Simplified cross-modal attention

        Args:
            vision_features: [batch_size, num_regions, vision_dim]
            language_features: [batch_size, seq_len, language_dim]

        Returns:
            attended_vision: [batch_size, num_regions, hidden_dim]
            attended_language: [batch_size, seq_len, hidden_dim]
        """
        batch_size, num_regions, _ = vision_features.shape
        _, seq_len, _ = language_features.shape

        # Project features to common space
        vision_proj = self.vision_proj(vision_features)  # [B, R, H]
        lang_proj = self.language_proj(language_features)  # [B, S, H]

        # Expand dimensions for cross-attention
        vision_expanded = vision_proj.unsqueeze(2).expand(-1, -1, seq_len, -1)  # [B, R, S, H]
        lang_expanded = lang_proj.unsqueeze(1).expand(-1, num_regions, -1, -1)  # [B, R, S, H]

        # Concatenate and compute attention scores
        combined = torch.cat([vision_expanded, lang_expanded], dim=-1)  # [B, R, S, 2*H]
        attention_scores = self.attention_weights(combined).squeeze(-1)  # [B, R, S]

        # Apply softmax to get attention weights
        vision_attention = F.softmax(attention_scores, dim=2)  # [B, R, S] - vision attends to language
        lang_attention = F.softmax(attention_scores.transpose(1, 2), dim=2)  # [B, S, R] - language attends to vision

        # Compute attended features
        attended_vision = torch.bmm(vision_attention, lang_proj)  # [B, R, H]
        attended_language = torch.bmm(lang_attention, vision_proj)  # [B, S, H]

        # Combine original and attended features
        attended_vision = self.output_proj(torch.cat([vision_proj, attended_vision], dim=-1))
        attended_language = self.output_proj(torch.cat([lang_proj, attended_language], dim=-1))

        return attended_vision, attended_language

class MultiModalFusion(nn.Module):
    """Main fusion module that combines vision, language, and action features"""

    def __init__(self, vision_dim: int, language_dim: int, action_dim: int, hidden_dim: int = 512):
        super().__init__()
        self.hidden_dim = hidden_dim

        # Cross-modal attention modules
        self.vision_language_attention = SimpleCrossModalAttention(vision_dim, language_dim, hidden_dim)
        self.vision_action_attention = SimpleCrossModalAttention(vision_dim, action_dim, hidden_dim)
        self.language_action_attention = SimpleCrossModalAttention(language_dim, action_dim, hidden_dim)

        # Self-attention for each modality
        self.vision_self_attention = nn.MultiheadAttention(hidden_dim, num_heads=8, dropout=0.1)
        self.language_self_attention = nn.MultiheadAttention(hidden_dim, num_heads=8, dropout=0.1)
        self.action_self_attention = nn.MultiheadAttention(hidden_dim, num_heads=4, dropout=0.1)

        # Final fusion layers
        self.fusion_layer = nn.Sequential(
            nn.Linear(hidden_dim * 3, hidden_dim * 2),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_dim, hidden_dim)
        )

        # Layer normalization
        self.norm = nn.LayerNorm(hidden_dim)

    def forward(self, vision_features: torch.Tensor,
                language_features: torch.Tensor,
                action_features: torch.Tensor) -> torch.Tensor:
        """
        Fuse vision, language, and action features through cross-modal attention

        Args:
            vision_features: [batch_size, num_regions, vision_dim]
            language_features: [batch_size, seq_len, language_dim]
            action_features: [batch_size, action_dim] or [batch_size, seq_len, action_dim]

        Returns:
            fused_features: [batch_size, hidden_dim]
        """
        batch_size = vision_features.size(0)

        # Apply cross-modal attention
        attended_vision_lang, attended_language_vision = self.vision_language_attention(
            vision_features, language_features
        )

        # Handle action features shape
        if action_features.dim() == 2:
            # If action features are [B, action_dim], expand to [B, 1, action_dim]
            action_features_expanded = action_features.unsqueeze(1).expand(-1, language_features.size(1), -1)
        else:
            action_features_expanded = action_features

        attended_vision_act, attended_action_vision = self.vision_action_attention(
            vision_features, action_features_expanded
        )

        attended_language_act, attended_action_language = self.language_action_attention(
            language_features, action_features_expanded
        )

        # Apply self-attention to refine representations
        # Vision self-attention
        vision_combined = attended_vision_lang + attended_vision_act
        vision_combined = vision_combined.transpose(0, 1)  # [num_regions, batch_size, hidden_dim]
        vision_self_attended, _ = self.vision_self_attention(vision_combined, vision_combined, vision_combined)
        vision_self_attended = vision_self_attended.transpose(0, 1)  # [batch_size, num_regions, hidden_dim]
        avg_vision = vision_self_attended.mean(dim=1)  # [batch_size, hidden_dim]

        # Language self-attention
        language_combined = attended_language_vision + attended_language_act
        language_combined = language_combined.transpose(0, 1)  # [seq_len, batch_size, hidden_dim]
        language_self_attended, _ = self.language_self_attention(language_combined, language_combined, language_combined)
        language_self_attended = language_self_attended.transpose(0, 1)  # [batch_size, seq_len, hidden_dim]
        avg_language = language_self_attended.mean(dim=1)  # [batch_size, hidden_dim]

        # Action self-attention
        action_combined = attended_action_vision + attended_action_language
        action_combined = action_combined.transpose(0, 1)  # [seq_len, batch_size, hidden_dim]
        action_self_attended, _ = self.action_self_attention(action_combined, action_combined, action_combined)
        action_self_attended = action_self_attended.transpose(0, 1)  # [batch_size, seq_len, hidden_dim]
        avg_action = action_self_attended.mean(dim=1)  # [batch_size, hidden_dim]

        # Concatenate all modality representations
        combined_features = torch.cat([avg_vision, avg_language, avg_action], dim=-1)  # [B, 3*hidden_dim]

        # Apply final fusion
        fused_features = self.fusion_layer(combined_features)
        fused_features = self.norm(fused_features)

        return fused_features

class LateFusionModule(nn.Module):
    """Late fusion module that combines modalities at the decision level"""

    def __init__(self, input_dims: List[int], output_dim: int, fusion_type: str = 'concat'):
        super().__init__()
        self.fusion_type = fusion_type
        self.input_dims = input_dims
        self.output_dim = output_dim

        if fusion_type == 'concat':
            total_input_dim = sum(input_dims)
            self.fusion_layer = nn.Sequential(
                nn.Linear(total_input_dim, output_dim),
                nn.ReLU(),
                nn.Dropout(0.2),
                nn.Linear(output_dim, output_dim)
            )
        elif fusion_type == 'attention':
            # Attention-based fusion
            self.modality_weights = nn.Parameter(torch.ones(len(input_dims)))
            self.fusion_layer = nn.Linear(sum(input_dims), output_dim)
        elif fusion_type == 'gated':
            # Gated fusion where each modality has learnable gate
            self.gate_layers = nn.ModuleList([
                nn.Linear(dim, output_dim) for dim in input_dims
            ])
            self.modality_gates = nn.Parameter(torch.ones(len(input_dims)))
        else:
            raise ValueError(f"Unknown fusion type: {fusion_type}")

    def forward(self, modalities: List[torch.Tensor]) -> torch.Tensor:
        """Fuse multiple modalities using specified fusion strategy"""
        if len(modalities) != len(self.input_dims):
            raise ValueError(f"Expected {len(self.input_dims)} modalities, got {len(modalities)}")

        if self.fusion_type == 'concat':
            # Concatenate all modalities
            fused_input = torch.cat(modalities, dim=-1)
            return self.fusion_layer(fused_input)

        elif self.fusion_type == 'attention':
            # Compute attention weights for each modality
            weights = F.softmax(self.modality_weights, dim=0)

            # Weighted sum of modalities
            weighted_modalities = []
            for i, modality in enumerate(modalities):
                # Average across sequence dimension if needed
                if modality.dim() > 2:
                    modality = modality.mean(dim=1)
                weighted_modalities.append(modality * weights[i])

            fused_input = torch.cat(weighted_modalities, dim=-1)
            return self.fusion_layer(fused_input)

        elif self.fusion_type == 'gated':
            # Gated fusion
            fused_modalities = []
            for i, (modality, gate_layer) in enumerate(zip(modalities, self.gate_layers)):
                # Average across sequence dimension if needed
                if modality.dim() > 2:
                    modality = modality.mean(dim=1)
                # Apply gate
                gated_output = gate_layer(modality) * torch.sigmoid(self.modality_gates[i])
                fused_modalities.append(gated_output)

            # Sum all gated modalities
            return torch.stack(fused_modalities, dim=0).sum(dim=0)

def main():
    """Test the cross-modal fusion implementation"""
    print("Testing Cross-Modal Fusion...")

    # Set random seed for reproducibility
    torch.manual_seed(42)

    # Define dimensions
    batch_size = 4
    num_regions = 10
    seq_len = 20
    vision_dim = 512
    language_dim = 300
    action_dim = 128
    hidden_dim = 256

    # Create sample features
    vision_features = torch.randn(batch_size, num_regions, vision_dim)
    language_features = torch.randn(batch_size, seq_len, language_dim)
    action_features = torch.randn(batch_size, action_dim)

    print(f"Vision features shape: {vision_features.shape}")
    print(f"Language features shape: {language_features.shape}")
    print(f"Action features shape: {action_features.shape}")

    # Test simple cross-modal attention
    print("\n--- Testing Simple Cross-Modal Attention ---")
    simple_attention = SimpleCrossModalAttention(vision_dim, language_dim, hidden_dim)

    attended_vision, attended_language = simple_attention(vision_features, language_features)
    print(f"Attended vision shape: {attended_vision.shape}")
    print(f"Attended language shape: {attended_language.shape}")

    # Test full multi-modal fusion
    print("\n--- Testing Multi-Modal Fusion ---")
    fusion_module = MultiModalFusion(vision_dim, language_dim, action_dim, hidden_dim)

    fused_output = fusion_module(vision_features, language_features, action_features)
    print(f"Fused output shape: {fused_output.shape}")

    # Test late fusion
    print("\n--- Testing Late Fusion ---")
    late_fusion = LateFusionModule([hidden_dim, hidden_dim, action_dim], hidden_dim, 'concat')

    # Use the attended features from cross-modal attention
    avg_vision = attended_vision.mean(dim=1)  # [B, hidden_dim]
    avg_language = attended_language.mean(dim=1)  # [B, hidden_dim]

    late_fused = late_fusion([avg_vision, avg_language, action_features])
    print(f"Late fused output shape: {late_fused.shape}")

    # Test different fusion types
    fusion_types = ['concat', 'attention', 'gated']
    for fusion_type in fusion_types:
        print(f"\n--- Testing {fusion_type} fusion ---")
        late_fusion_type = LateFusionModule([hidden_dim, hidden_dim, action_dim], hidden_dim, fusion_type)
        fused_result = late_fusion_type([avg_vision, avg_language, action_features])
        print(f"{fusion_type} fusion output shape: {fused_result.shape}")

if __name__ == "__main__":
    main()