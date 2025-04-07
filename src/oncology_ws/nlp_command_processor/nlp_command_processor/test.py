import spacy


nlp = spacy.load("en_core_web_sm")  # Load English model



location_tokens = []
for i, token in enumerate(doc):
    if token.pos_ in ["NOUN", "PROPN"]:  # Nouns or proper nouns (e.g., "kitchen", "living room")
        location_tokens.append(token.text)

print(" ".join(location_tokens))  # Output: living room
