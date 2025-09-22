// BudgetRangeQuestion.js
import MultiSlider from "@ptomasroos/react-native-multi-slider";
import React, { useEffect, useMemo, useState } from "react";
import { Dimensions, StyleSheet, Text, View } from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { QUIZ as QUIZ_SCHEMA } from "../../../../../components/Interests_lifestyle";

const { width } = Dimensions.get("window");

// ✅ Use the budget node from your schema
const NODE = QUIZ_SCHEMA.sections.basics.questions.budget_range;
// { type:"slider-range", min, max, step, default:[low,high], format:"currency" }

const currency = (n) =>
  `$${Math.round(n).toString().replace(/\B(?=(\d{3})+(?!\d))/g, ",")}`;

export default function BudgetRangeQuestion({ selected, onSelect }) {
  const min = NODE.min ?? 300;
  const max = NODE.max ?? 5000;
  const step = NODE.step ?? 25;
  const def = Array.isArray(selected) && selected.length === 2
    ? selected
    : NODE.default ?? [900, 1400];


    const [values, setValues] = useState([def[0], def[1]]);

    useEffect(() => {
      if (!Array.isArray(selected) || selected.length !== 2) {
        onSelect?.([def[0], def[1]]);
      }
      // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

  // clamp & snap to step
  const snapped = useMemo(() => {
    const snap = (v) => Math.min(max, Math.max(min, Math.round(v / step) * step));
    return [snap(values[0]), snap(values[1])];
  }, [values, min, max, step]);

  return (
    <SafeAreaView style={styles.safeArea}>
      <View style={styles.card}>
        <Text style={styles.title}>{NODE.label || "Monthly rent budget"}</Text>

        <View style={styles.pill}>
          <Text style={styles.pillText}>
            {currency(snapped[0])} – {currency(snapped[1])}/mo
          </Text>
        </View>

        <View style={styles.sliderWrap}>
          <MultiSlider
            values={snapped}
            onValuesChange={(vals) => setValues([vals[0], vals[1]])}
            onValuesChangeFinish={(vals) => onSelect?.([vals[0], vals[1]])}
            min={min}
            max={max}
            step={step}
            allowOverlap={false}
            snapped
            containerStyle={{ width: "100%" }}
            trackStyle={{ height: 6, backgroundColor: "#E5E7EB" }}
            selectedStyle={{ backgroundColor: "#3B82F6" }}
            markerStyle={{
              height: 24, width: 24, borderRadius: 12, backgroundColor: "#3B82F6",
              borderWidth: 2, borderColor: "white", elevation: 3,
            }}
          />
          <View style={styles.scaleRow}>
            <Text style={styles.scaleLabel}>{currency(min)}</Text>
            <Text style={styles.scaleLabel}>{currency(max)}</Text>
          </View>
        </View>

        {NODE.format === "currency" ? (
          <Text style={styles.helper}>Drag both ends to set your monthly rent range.</Text>
        ) : null}
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
  title: { fontSize: 18, fontWeight: "700", color: "#22223B", marginBottom: 12 },
  pill: {
    alignSelf: "flex-start", backgroundColor: "#EEF2FF", borderColor: "#DBEAFE", borderWidth: 1,
    borderRadius: 999, paddingVertical: 6, paddingHorizontal: 12, marginBottom: 12,
  },
  pillText: { color: "#1D4ED8", fontWeight: "700" },
  sliderWrap: { marginTop: 8 },
  scaleRow: { flexDirection: "row", justifyContent: "space-between", marginTop: 8 },
  scaleLabel: { fontSize: 12, color: "#6B7280" },
  helper: { marginTop: 10, fontSize: 12, color: "#6B7280" },
});
