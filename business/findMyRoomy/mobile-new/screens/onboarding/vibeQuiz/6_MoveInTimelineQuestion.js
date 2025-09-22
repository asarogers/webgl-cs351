// 6_MoveInTimelineQuestion.js
import React, { useEffect, useMemo, useState } from "react";
import {
  View,
  Text,
  TouchableOpacity,
  StyleSheet,
  Animated,
  Modal,
  Pressable,
  Platform,
} from "react-native";
import { SafeAreaView } from 'react-native-safe-area-context';
import { Ionicons } from "@expo/vector-icons";
import DateTimePicker from "@react-native-community/datetimepicker";
import { QUIZ as quiz_options } from "../../../components/Interests_lifestyle"; // ⬅️ keyed schema

// pull the question from the schema
const q = quiz_options.sections.basics.questions.move_in_timeline;

// optional UI metadata per option id
/** @type {{[k:string]: {icon:string, color:string, description?:string} }} */
const META = {
  asap_2w: { icon: "⚡", color: "#EF4444", description: "Ready to move within 2 weeks" },
  "1m":    { icon: "📅", color: "#F59E0B", description: "Within 1 month" },
  "2m":    { icon: "🗓️", color: "#10B981", description: "1–2 months" },
  "2p":    { icon: "⏳", color: "#3B82F6", description: "2+ months" },
  specific:{ icon: "📍", color: "#8B5CF6", description: "Let me pick an exact move-in date" },
};

export default function MoveInTimelineQuestion({
  selected,          // one of "asap_2w" | "1m" | "2m" | "2p" | "specific"
  onSelect,          // (id) => void
  specificDate,      // Date | string | null
  onDateSelect,      // (date: Date) => void
}) {
  const [fadeAnim] = useState(new Animated.Value(0));
  const [slideAnim] = useState(new Animated.Value(30));
  const [buttonAnimations] = useState(
    q.optionsOrder.map(() => new Animated.Value(0))
  );
  const [showDateModal, setShowDateModal] = useState(false);
  const [tempDate, setTempDate] = useState(
    specificDate ? new Date(specificDate) : new Date()
  );

  useEffect(() => {
    Animated.parallel([
      Animated.timing(fadeAnim, { toValue: 1, duration: 600, useNativeDriver: true }),
      Animated.timing(slideAnim, { toValue: 0, duration: 600, useNativeDriver: true }),
    ]).start();

    const seq = buttonAnimations.map((anim, i) =>
      Animated.timing(anim, { toValue: 1, duration: 400, delay: i * 100, useNativeDriver: true })
    );
    Animated.stagger(100, seq).start();
  }, []);

  const formatDate = (input) => {
    if (!input) return "";
    const d = input instanceof Date ? input : new Date(input);
    if (isNaN(d.getTime())) return "";
    return d.toLocaleDateString("en-US", { month: "short", day: "numeric", year: "numeric" });
  };

  // Build options array from schema (and inject pretty date for "specific")
  const options = useMemo(() => {
    const pretty = specificDate ? formatDate(specificDate) : "";
    return q.optionsOrder.map((id) => {
      const base = { id, label: q.options[id].label };
      const meta = META[id] ?? { icon: "🗓️", color: "#3B82F6" };
      const description =
        id === "specific" && specificDate
          ? `Move-in: ${pretty}`
          : (meta.description ?? "");
      return { ...base, ...meta, description };
    });
  }, [specificDate]);

  const openDatePicker = () => {
    setTempDate(specificDate ? new Date(specificDate) : new Date());
    setShowDateModal(true);
  };

  const handleSelect = (id) => {
    if (id === "specific") {
      openDatePicker();
    }
    onSelect?.(id);
  };

  const confirmDate = () => {
    onDateSelect?.(tempDate);
    onSelect?.("specific");
    setShowDateModal(false);
  };

  return (
    <SafeAreaView style={styles.container}>
      <Animated.View style={[styles.content, { opacity: fadeAnim, transform: [{ translateY: slideAnim }] }]}>
        <View style={styles.card}>
          <View style={styles.questionHeader}>
            <View style={styles.questionIcon}>
              <Ionicons name="calendar-outline" size={24} color="#3B82F6" />
            </View>
            <Text style={styles.questionText}>When are you looking to move?</Text>
            <Text style={styles.questionSubtext}>This helps us prioritize listings for you</Text>
          </View>

          <View style={styles.optionsContainer}>
            {options.map((opt, index) => (
              <Animated.View
                key={`${opt.id}-${opt.description}`} // re-render when "specific" description changes
                style={{
                  opacity: buttonAnimations[index],
                  transform: [
                    { translateY: buttonAnimations[index].interpolate({ inputRange: [0, 1], outputRange: [20, 0] }) },
                    { scale: buttonAnimations[index] },
                  ],
                }}
              >
                <TouchableOpacity
                  onPress={() => handleSelect(opt.id)}
                  style={[
                    styles.optionButton,
                    selected === opt.id && [
                      styles.optionButtonSelected,
                      { borderColor: opt.color, backgroundColor: `${opt.color}10` },
                    ],
                  ]}
                  activeOpacity={0.8}
                >
                  <View style={styles.optionContent}>
                    <View
                      style={[
                        styles.emojiContainer,
                        { backgroundColor: `${opt.color}20` },
                        selected === opt.id && { backgroundColor: `${opt.color}30`, transform: [{ scale: 1.1 }] },
                      ]}
                    >
                      <Text style={styles.emoji}>{opt.icon}</Text>
                    </View>

                    <View style={styles.optionTextContainer}>
                      <Text style={[styles.optionLabel, selected === opt.id && { color: opt.color, fontWeight: "700" }]}>
                        {opt.label}
                      </Text>

                      {!!opt.description && (
                        <Text
                          key={opt.description}
                          style={[
                            styles.optionDescription,
                            opt.id === "specific" && specificDate && { fontWeight: "600", color: "#8B5CF6" },
                          ]}
                        >
                          {opt.description}
                        </Text>
                      )}
                    </View>

                    {selected === opt.id && (
                      <Animated.View style={[styles.checkContainer, { backgroundColor: opt.color }]}>
                        <Ionicons name="checkmark" size={16} color="white" />
                      </Animated.View>
                    )}
                  </View>
                </TouchableOpacity>
              </Animated.View>
            ))}
          </View>
        </View>
      </Animated.View>

      {/* Bottom-sheet modal date picker */}
      <Modal visible={showDateModal} animationType="slide" transparent onRequestClose={() => setShowDateModal(false)}>
        <Pressable style={styles.backdrop} onPress={() => setShowDateModal(false)} />
        <View style={styles.sheet}>
          <View style={styles.sheetHeader}>
            <Text style={styles.sheetTitle}>Select move-in date</Text>
            <Text style={styles.sheetSubtitle}>{formatDate(tempDate)}</Text>
          </View>

          <View style={styles.pickerContainer}>
            <DateTimePicker
              value={tempDate}
              mode="date"
              display={Platform.OS === "ios" ? "inline" : "calendar"}
              minimumDate={new Date()}
              maximumDate={new Date(Date.now() + 365 * 24 * 60 * 60 * 1000)}
              onChange={(_, d) => d && setTempDate(d)}
            />
          </View>

          <View style={styles.sheetActions}>
            <TouchableOpacity style={[styles.actionBtn, styles.cancelBtn]} onPress={() => setShowDateModal(false)}>
              <Text style={styles.cancelText}>Cancel</Text>
            </TouchableOpacity>
            <TouchableOpacity style={[styles.actionBtn, styles.confirmBtn]} onPress={confirmDate}>
              <Text style={styles.confirmText}>Use this date</Text>
            </TouchableOpacity>
          </View>
        </View>
      </Modal>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: { backgroundColor: "#F8FAFC", flex: 1 },
  content: { flex: 1, paddingHorizontal: 0, paddingTop: 0, justifyContent: "center" },
  card: {
    marginTop: 0,
    backgroundColor: "white",
    borderRadius: 24,
    padding: 10,
    shadowColor: "#000",
    shadowOpacity: 0.1,
    shadowOffset: { width: 0, height: 8 },
    shadowRadius: 24,
    elevation: 12,
    marginHorizontal: 0,
    maxHeight: "85%",
  },
  questionHeader: { alignItems: "center", marginBottom: 32 },
  questionIcon: {
    width: 56, height: 56, borderRadius: 28, backgroundColor: "#EEF2FF",
    justifyContent: "center", alignItems: "center", marginBottom: 20,
  },
  questionText: { fontSize: 24, fontWeight: "800", color: "#111827", textAlign: "center", marginBottom: 12, lineHeight: 32 },
  questionSubtext: { fontSize: 16, color: "#64748B", textAlign: "center", lineHeight: 20 },
  optionsContainer: { gap: 8, paddingBottom: 20 },
  optionButton: { borderRadius: 20, borderWidth: 2, borderColor: "#E2E8F0", backgroundColor: "#FFFFFF", overflow: "hidden", minHeight: 88 },
  optionButtonSelected: {
    borderWidth: 2, shadowColor: "#3B82F6", shadowOpacity: 0.15, shadowOffset: { width: 0, height: 4 },
    shadowRadius: 12, elevation: 6, transform: [{ scale: 1.02 }],
  },
  optionContent: { flexDirection: "row", alignItems: "center", padding: 20 },
  emojiContainer: { width: 48, height: 48, borderRadius: 24, justifyContent: "center", alignItems: "center", marginRight: 16 },
  emoji: { fontSize: 22 },
  optionTextContainer: { flex: 1 },
  optionLabel: { fontSize: 17, fontWeight: "600", color: "#111827", marginBottom: 6, lineHeight: 24 },
  optionDescription: { fontSize: 14, color: "#64748B", lineHeight: 20 },
  checkContainer: { width: 28, height: 28, borderRadius: 14, justifyContent: "center", alignItems: "center", marginLeft: 12 },

  // Modal
  backdrop: { flex: 1, backgroundColor: "rgba(0,0,0,0.35)" },
  sheet: {
    position: "absolute",
    left: 0, right: 0, bottom: 0,
    backgroundColor: "#fff",
    borderTopLeftRadius: 20, borderTopRightRadius: 20,
    paddingBottom: 24, paddingHorizontal: 16, paddingTop: 12,
    maxHeight: "85%",
  },
  sheetHeader: { alignItems: "center", marginBottom: 8 },
  sheetTitle: { fontSize: 18, fontWeight: "700", color: "#111827" },
  sheetSubtitle: { fontSize: 14, color: "#64748B", marginTop: 4 },
  pickerContainer: { paddingVertical: 8 },
  sheetActions: { flexDirection: "row", justifyContent: "flex-end", gap: 12, paddingTop: 8 },
  actionBtn: { paddingVertical: 12, paddingHorizontal: 16, borderRadius: 10 },
  cancelBtn: { backgroundColor: "#F1F5F9" },
  confirmBtn: { backgroundColor: "#8B5CF6" },
  cancelText: { color: "#111827", fontWeight: "600" },
  confirmText: { color: "#fff", fontWeight: "700" },
});
