---
sidebar_position: 2
title: "وژن ایکسیمپلز"
---

# وژن ایکسیمپلز

وژن لینگویج ایکشن سسٹم میں وژن اور لینگویج کے مابین انٹرایکشن کو ظاہر کرنے والے ایکسیمپلز۔ یہ سیکشن مختلف وژن اور لینگویج کے امتزاج کو دکھاتا ہے۔

## بیسک انٹرایکشن ایکسیمپلز

### 1. آبجیکٹ کی نشاندہی

#### کمانڈ:
"وہ لال گیند کو اٹھاؤ جو میز پر ہے"

#### وژن کا جواب:
- لال رنگ کی شناخت
- گیند کی شکل کی تصدیق
- میز کے اوپر پوزیشن کی تلاش

#### لینگویج انٹرپری ٹیشن:
- "لال" → رنگ کی تلاش
- "گیند" → شکل کی تلاش
- "میز پر" → مقام کی تلاش

#### کوڈ مثال:
```python
def process_visual_command(self, text_command, vision_data):
    """وژن اور ٹیکسٹ کمانڈ کو مربوط کریں"""
    # ٹیکسٹ کو پارس کریں
    parsed_command = self.parse_text_command(text_command)

    # وژن ڈیٹا میں سے متعلقہ آبجیکٹس تلاش کریں
    relevant_objects = self.find_relevant_objects(
        vision_data['detections'],
        parsed_command['attributes']
    )

    # مطلوبہ آبجیکٹ کو فلٹر کریں
    target_object = self.filter_target_object(
        relevant_objects,
        parsed_command['spatial_relationships']
    )

    return target_object

def find_relevant_objects(self, detections, attributes):
    """خصوصیات کے مطابق آبجیکٹس تلاش کریں"""
    relevant = []
    for detection in detections:
        matches = True

        # رنگ کی تلاش
        if 'color' in attributes:
            color = self.get_object_color(detection)
            if color != attributes['color']:
                matches = False

        # شکل کی تلاش
        if 'shape' in attributes:
            shape = self.get_object_shape(detection)
            if shape != attributes['shape']:
                matches = False

        if matches:
            relevant.append(detection)

    return relevant
```

### 2. سپیشل ریلیشن شپس

#### کمانڈ:
"بائیں طرف کے کپ کو اٹھاؤ"

#### وژن کا جواب:
- کئی کپس کی شناخت
- بائیں طرف کا تعین
- اسپیسیل ریلیشن شپ کا حساب

#### کوڈ مثال:
```python
def find_spatially_related_object(self, detections, spatial_descriptor):
    """سپیشل ڈیسکرپٹر کے مطابق آبجیکٹ تلاش کریں"""
    # آبجیکٹس کو X-axis کے حساب سے ترتیب دیں
    sorted_objects = sorted(detections, key=lambda x: x['bbox']['center_x'])

    if spatial_descriptor == 'left':
        # سب سے بائیں والے کو منتخب کریں
        return sorted_objects[0] if sorted_objects else None
    elif spatial_descriptor == 'right':
        # سب سے دائیں والے کو منتخب کریں
        return sorted_objects[-1] if sorted_objects else None
    elif spatial_descriptor == 'center':
        # درمیان والے کو منتخب کریں
        center_idx = len(sorted_objects) // 2
        return sorted_objects[center_idx] if sorted_objects else None
    elif spatial_descriptor == 'front':
        # سب سے اوپر والے (Y-axis) کو منتخب کریں
        sorted_by_y = sorted(detections, key=lambda x: x['bbox']['center_y'])
        return sorted_by_y[0] if sorted_by_y else None
    elif spatial_descriptor == 'back':
        # سب سے نیچے والے (Y-axis) کو منتخب کریں
        sorted_by_y = sorted(detections, key=lambda x: x['bbox']['center_y'], reverse=True)
        return sorted_by_y[0] if sorted_by_y else None

    return None
```

### 3. کوانٹیٹی بیسڈ سلیکشن

#### کمانڈ:
"تیسرے نیلے کپ کو اٹھاؤ"

#### وژن کا جواب:
- تمام نیلے کپس کی شناخت
- تیسرے نمبر کا تعین
- ترتیب کے مطابق منتخب کرنا

#### کوڈ مثال:
```python
def find_quantified_object(self, detections, attributes, quantity):
    """مقدار کے مطابق آبجیکٹ تلاش کریں"""
    # خصوصیات کے مطابق فلٹر کریں
    matching_objects = self.find_relevant_objects(detections, attributes)

    # ترتیب دیں (مثلاً بائیں سے دائیں)
    ordered_objects = sorted(matching_objects, key=lambda x: x['bbox']['center_x'])

    # مخصوص انڈیکس کا آبجیکٹ منتخب کریں
    if quantity <= len(ordered_objects):
        return ordered_objects[quantity - 1]  # 1-based indexing
    else:
        return None  # مطلوبہ مقدار دستیاب نہیں
```

## کمپلیکس انٹرایکشن ایکسیمپلز

### 1. کنٹیکسٹ اوار کمانڈز

#### کمانڈ:
"وہی چیز اٹھاؤ جسے میں نے ابھی دکھایا تھا"

#### وژن کا جواب:
- پچھلے کنٹیکسٹ کو یاد رکھنا
- حالیہ ڈیٹیکشنز کو تلاش کرنا
- اسی آبجیکٹ کو دوبارہ شناخت کرنا

#### کوڈ مثال:
```python
class ContextAwareLanguageProcessor:
    def __init__(self):
        self.context_history = []
        self.max_context_length = 10

    def process_contextual_command(self, text_command, vision_data, context_reference):
        """کنٹیکسٹ کے حوالے سے کمانڈ پروسیس کریں"""
        if context_reference == 'previous_object':
            # پچھلا آبجیکٹ حاصل کریں
            previous_object = self.get_previous_object()
            if previous_object:
                # اسی آبجیکٹ کو موجودہ وژن ڈیٹا میں تلاش کریں
                target_object = self.match_to_current_detection(
                    previous_object, vision_data['detections']
                )
                return target_object

        elif context_reference == 'demonstrated_object':
            # ڈیمو سے آبجیکٹ حاصل کریں
            demonstrated_object = self.get_demonstrated_object()
            if demonstrated_object:
                # موجودہ وژن ڈیٹا میں میچ کریں
                target_object = self.match_to_current_detection(
                    demonstrated_object, vision_data['detections']
                )
                return target_object

        return None

    def get_previous_object(self):
        """پچھلا آبجیکٹ حاصل کریں"""
        if self.context_history:
            return self.context_history[-1]
        return None

    def match_to_current_detection(self, reference_object, current_detections):
        """ریفرنس آبجیکٹ کو موجودہ ڈیٹیکشنز سے میچ کریں"""
        for detection in current_detections:
            similarity = self.calculate_object_similarity(
                reference_object, detection
            )
            if similarity > 0.8:  # 80% تشابہ کی شرط
                return detection
        return None
```

### 2. ایکشن بیسڈ وژن

#### کمانڈ:
"وہی جگہ پر چیز رکھو جہاں میں نے پہلے رکھا تھا"

#### وژن کا جواب:
- پچھلی پوزیشن کو یاد رکھنا
- موجودہ ماحول کا تجزیہ
- اسی جگہ کو شناخت کرنا

#### کوڈ مثال:
```python
def process_action_reference_command(self, text_command, current_vision_data, action_reference):
    """ایکشن ریفرنس کے مطابق کمانڈ پروسیس کریں"""
    if action_reference == 'previous_location':
        # پچھلا لوکیشن حاصل کریں
        previous_location = self.get_previous_location()
        if previous_location:
            # موجودہ ماحول میں اسی جگہ کو شناخت کریں
            target_location = self.identify_similar_location(
                previous_location, current_vision_data
            )
            return target_location

    return None

def identify_similar_location(self, reference_location, current_vision_data):
    """ریفرنس لوکیشن کے مماثل لوکیشن تلاش کریں"""
    # سپیشل ریلیشن شپس کا تجزیہ کریں
    # مثلاً 'میز کے اوپر'، 'دروازے کے پاس' وغیرہ
    for landmark in current_vision_data['landmarks']:
        if self.is_similar_location(landmark, reference_location):
            return landmark

    return None
```

## وژن-لینگویج فیوژن الگورتھم

### کراس ماڈل اتنشن:

```python
import torch
import torch.nn as nn

class VisionLanguageFusion(nn.Module):
    def __init__(self, vision_dim=512, language_dim=512, fusion_dim=512):
        super(VisionLanguageFusion, self).__init__()

        # وژن اور لینگویج انکوڈرز
        self.vision_encoder = nn.Linear(vision_dim, fusion_dim)
        self.language_encoder = nn.Linear(language_dim, fusion_dim)

        # کراس ماڈل اتنشن
        self.cross_attention = nn.MultiheadAttention(
            embed_dim=fusion_dim,
            num_heads=8
        )

        # فیوژن لیئر
        self.fusion_layer = nn.Sequential(
            nn.Linear(fusion_dim * 2, fusion_dim),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(fusion_dim, fusion_dim)
        )

        self.norm = nn.LayerNorm(fusion_dim)

    def forward(self, vision_features, language_features):
        """وژن اور لینگویج فیچرز کو مربوط کریں"""
        # فیچرز کو انکوڈ کریں
        vision_encoded = self.vision_encoder(vision_features)
        lang_encoded = self.language_encoder(language_features)

        # کراس اتنشن
        # وژن فیچرز کو لینگویج کے ذریعے اٹینڈ کریں
        attended_vision, _ = self.cross_attention(
            vision_encoded, lang_encoded, lang_encoded
        )

        # لینگویج فیچرز کو وژن کے ذریعے اٹینڈ کریں
        attended_language, _ = self.cross_attention(
            lang_encoded, vision_encoded, vision_encoded
        )

        # فیوژن
        combined_features = torch.cat([
            attended_vision, attended_language
        ], dim=-1)

        fused_output = self.fusion_layer(combined_features)
        fused_output = self.norm(fused_output + vision_encoded)

        return fused_output
```

## ٹیسٹنگ اور والیڈیشن

### ٹیسٹ کیسز:

```python
def test_vision_language_fusion():
    """وژن لینگویج فیوژن کو ٹیسٹ کریں"""
    # ٹیسٹ ڈیٹا تیار کریں
    vision_features = torch.randn(1, 10, 512)  # 10 آبجیکٹس، 512 dim
    language_features = torch.randn(1, 20, 512)  # 20 الفاظ، 512 dim

    # فیوژن ماڈل
    fusion_model = VisionLanguageFusion()

    # فیوژن کریں
    fused_output = fusion_model(vision_features, language_features)

    # چیک کریں کہ آؤٹ پٹ صحیح سائز کا ہے
    assert fused_output.shape == (1, 10, 512), "Fused output shape mismatch"

    print("Vision-Language fusion test passed!")

def test_spatial_reasoning():
    """سپیشل ریزننگ کو ٹیسٹ کریں"""
    # ڈیٹیکشنز کے ساتھ ٹیسٹ کریں
    detections = [
        {'bbox': {'center_x': 100, 'center_y': 200}, 'class': 'cup', 'color': 'blue'},
        {'bbox': {'center_x': 300, 'center_y': 200}, 'class': 'cup', 'color': 'red'},
        {'bbox': {'center_x': 500, 'center_y': 200}, 'class': 'cup', 'color': 'blue'}
    ]

    processor = ContextAwareLanguageProcessor()

    # بائیں طرف کا نیلا کپ تلاش کریں
    left_blue_cup = processor.find_spatially_related_object(
        [d for d in detections if d['color'] == 'blue'], 'left'
    )

    # چیک کریں کہ سب سے بائیں نیلا کپ منتخب ہوا
    assert left_blue_cup['bbox']['center_x'] == 100, "Leftmost blue cup not selected"

    print("Spatial reasoning test passed!")
```

## کارکردگی کے اشاریے

### ایکویسی:
- کمانڈ سمجھنے کی شرح: > 90%
- آبجیکٹ سلیکشن کی درستگی: > 85%
- سپیشل ریلیشن کی درستگی: > 80%

### ریسپانس ٹائم:
- وژن-لینگویج فیوژن: < 100ms
- کمانڈ پروسیسنگ: < 200ms
- کل سسٹم ریسپانس: < 500ms

یہ ایکسیمپلز وژن اور لینگویج کے مابین مؤثر انٹرایکشن کے طریقے کو ظاہر کرتے ہیں جو VLA سسٹم کے لیے اہم ہے۔