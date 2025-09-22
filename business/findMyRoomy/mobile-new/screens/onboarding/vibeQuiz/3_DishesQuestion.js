// TidinessQuestion.tsx
import React from "react";
import { View, Text, TouchableOpacity, StyleSheet, Dimensions } from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { QUIZ as QUIZ_SCHEMA } from "../../../components/Interests_lifestyle";

const { width } = Dimensions.get("window");

// ✅ Use the tidiness node (scale)
const NODE = QUIZ_SCHEMA.sections.lifestyle.questions.cleanliness_level;
// { type: "scale", min: 1, max: 5, anchors: { left: "Lived-in", right: "Spotless" } }


export default function TidinessQuestion({ selected, onSelect }) {
  const min = NODE.min ?? 1;
  const max = NODE.max ?? 5;
  const steps = Array.from({ length: max - min + 1 }, (_, i) => i + min);
  const left = NODE.anchors?.left ?? "Lower";
  const right = NODE.anchors?.right ?? "Higher";

  return (
    <SafeAreaView style={styles.safeArea}>
      <View style={styles.card}>
        <Text style={styles.questionText}>{NODE.label || "Cleanliness standard (shared spaces)"}</Text>

        {/* Anchors */}
        <View style={styles.anchorsRow}>
          <Text style={styles.anchorText}>{left}</Text>
          <Text style={styles.anchorText}>{right}</Text>
        </View>

        {/* Scale buttons */}
        <View style={styles.scaleRow}>
          {steps.map((n) => {
            const isSelected = selected === n;
            return (
              <TouchableOpacity
                key={n}
                style={[styles.step, isSelected && styles.stepSelected]}
                onPress={() => onSelect?.(n)}
                activeOpacity={0.8}
              >
                <Text style={[styles.stepText, isSelected && styles.stepTextSelected]}>{n}</Text>
              </TouchableOpacity>
            );
          })}
        </View>

        {/* Helper text */}
        <Text style={styles.helperText}>
          Tap a number closer to "{right}" if you like things extra tidy.
        </Text>
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: { flex: 1, backgroundColor: "#F8FAFC", alignItems: "center", justifyContent: "center" },
  card: {
    width: width * 0.92, backgroundColor: "#fff", borderRadius: 22, padding: 20,
    shadowColor: "#000", shadowOpacity: 0.06, shadowOffset: { width: 0, height: 6 }, shadowRadius: 24, elevation: 6,
  },
  questionText: { fontSize: 18, fontWeight: "700", color: "#22223B", marginBottom: 14, textAlign: "left" },
  anchorsRow: { flexDirection: "row", justifyContent: "space-between", marginBottom: 8 },
  anchorText: { fontSize: 13, color: "#6B7280" },
  scaleRow: { flexDirection: "row", justifyContent: "space-between", marginVertical: 8 },
  step: {
    width: 48, height: 48, borderRadius: 12, backgroundColor: "#F6F8FA",
    alignItems: "center", justifyContent: "center", borderWidth: 2, borderColor: "transparent",
  },
  stepSelected: { backgroundColor: "#EEF2FF", borderColor: "#3B82F6", shadowColor: "#3B82F6", shadowOpacity: 0.13,
    shadowOffset: { width: 0, height: 2 }, shadowRadius: 10, elevation: 4 },
  stepText: { fontSize: 16, color: "#374151", fontWeight: "600" },
  stepTextSelected: { color: "#3B82F6", fontWeight: "700" },
  helperText: { marginTop: 10, fontSize: 12, color: "#6B7280" },
});
