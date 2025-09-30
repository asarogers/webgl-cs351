import React, { useMemo } from "react";
import { StyleSheet, Text, View, TouchableOpacity } from "react-native";

/* ============================================================================
 * TYPES & INTERFACES
 * ========================================================================== */
/* ============================================================================
 * TYPES & INTERFACES
 * ========================================================================== */
interface ProfileData {
  name?: string;
  about?: string;
  photos?: { id: string; uri: string }[];
  interests?: string[];
  budget_min?: number;
  budget_max?: number;
  location?: string;
  rawZipcode?: string;
  location_sharing?: boolean;

  lifestyle?: {
    nonSmoker?: boolean;
    sleep_schedule?: string;
    dish_washing?: string;
    friends_over?: string;
  };

  substances?: {
    smoking_policy?: string;
    alcohol_use?: string;
    cannabis_policy?: string;
    substance_free_preference?: boolean;
  };

  pets?: {
    pet_ownership?: (string | { other: string })[];
    pet_tolerance?: (string | { other: string })[];
    pet_allergies?: (string | { other: string })[];
  };

  amenities?: {
    furnishing?: string;
    extras?: string[];
    parking?: string;
    laundry?: string;
    bathroom_pref?: string;
  };
}

interface ProfileStrengthItem {
  id: string;
  label: string;
  completed: boolean;
  priority: "high" | "medium" | "low";
}

/* ============================================================================
 * CALCULATION LOGIC
 * ========================================================================== */
export function computeProfileStrength(profile: ProfileData | null | undefined): {
  score: number;
  percentage: number;
  items: ProfileStrengthItem[];
  nextAction: ProfileStrengthItem | null;
} {
  if (!profile) return { score: 0, percentage: 0, items: [], nextAction: null };

  const items: ProfileStrengthItem[] = [
    // 🔑 Core Identity
    {
      id: "name",
      label: "Full name",
      completed: Boolean(profile.name),
      priority: "high",
    },
    {
      id: "photos",
      label: "Profile photos",
      completed: Array.isArray(profile.photos) && profile.photos.length > 0,
      priority: "high",
    },
    {
      id: "about",
      label: "About section (20+ characters)",
      completed: Boolean(profile.about && profile.about.trim().length > 20),
      priority: "high",
    },

    // 🎯 Interests & Budget
    {
      id: "interests",
      label: "Interests (3+ selected)",
      completed: Array.isArray(profile.interests) && profile.interests.length >= 3,
      priority: "medium",
    },
    {
      id: "budget",
      label: "Budget information",
      completed: Boolean(profile.budget_min && profile.budget_max),
      priority: "medium",
    },

    // 🏠 Lifestyle
    {
      id: "nonSmoker",
      label: "Non-smoker",
      completed: Boolean(profile.lifestyle?.nonSmoker),
      priority: "medium",
    },
   

    // 📍 Location
    {
      id: "location",
      label: "Location details",
      completed: Boolean(profile.location_sharing && profile.rawZipcode),
      priority: "medium",
    },

    // 🚬 Substances
    {
      id: "smoking_policy",
      label: "Smoking policy",
      completed: Boolean(profile.substances?.smoking_policy),
      priority: "low",
    },
    {
      id: "alcohol_use",
      label: "Alcohol policy",
      completed: Boolean(profile.substances?.alcohol_use),
      priority: "low",
    },
    {
      id: "cannabis_policy",
      label: "Cannabis policy",
      completed: Boolean(profile.substances?.cannabis_policy),
      priority: "low",
    },

    // 🐶 Pets
    {
      id: "pet_ownership",
      label: "Pet ownership",
      completed: Array.isArray(profile.pets?.pet_ownership) && profile.pets?.pet_ownership.length > 0,
      priority: "low",
    },
    {
      id: "pet_tolerance",
      label: "Comfort with roommate's pets",
      completed: Array.isArray(profile.pets?.pet_tolerance) && profile.pets?.pet_tolerance.length > 0,
      priority: "low",
    },
    {
      id: "pet_allergies",
      label: "Pet allergies",
      completed: Array.isArray(profile.pets?.pet_allergies) && profile.pets?.pet_allergies.length > 0,
      priority: "low",
    },

    // 🛋 Amenities
    {
      id: "furnishing",
      label: "Furnishing preference",
      completed: Boolean(profile.amenities?.furnishing),
      priority: "low",
    },
    {
      id: "extras",
      label: "Nice-to-haves",
      completed: Array.isArray(profile.amenities?.extras) && profile.amenities?.extras.length > 0,
      priority: "low",
    },
    {
      id: "parking",
      label: "Parking preference",
      completed: Boolean(profile.amenities?.parking),
      priority: "low",
    },
    {
      id: "laundry",
      label: "Laundry preference",
      completed: Boolean(profile.amenities?.laundry),
      priority: "low",
    },
    {
      id: "bathroom_pref",
      label: "Bathroom preference",
      completed: Boolean(profile.amenities?.bathroom_pref),
      priority: "low",
    },
  ];

  const completedCount = items.filter((item) => item.completed).length;
  const percentage = Math.round((completedCount / items.length) * 100);

  // Find next recommended action (prioritize high > medium > low)
  const incompleteItems = items.filter((item) => !item.completed);
  const nextAction = 
    incompleteItems.find((item) => item.priority === "high") ||
    incompleteItems.find((item) => item.priority === "medium") ||
    incompleteItems.find((item) => item.priority === "low") ||
    null;

  return {
    score: completedCount,
    percentage,
    items,
    nextAction,
  };
}



/* ============================================================================
 * STRENGTH LEVEL CONFIGURATION
 * ========================================================================== */
function getStrengthLevel(percentage: number) {
  if (percentage >= 80) return { level: 'Excellent', color: '#10B981', bgColor: '#064E3B' };
  if (percentage >= 60) return { level: 'Good', color: '#F59E0B', bgColor: '#451A03' };
  if (percentage >= 40) return { level: 'Fair', color: '#EF4444', bgColor: '#7F1D1D' };
  return { level: 'Getting Started', color: '#6B7280', bgColor: '#1F2937' };
}

/* ============================================================================
 * HOOKS
 * ========================================================================== */
export function useProfileStrengthDots(percentage: number, total: number = 5) {
  return useMemo(() => {
    const filled = Math.round((percentage / 100) * total);
    return Array.from({ length: total }, (_, i) => i < filled);
  }, [percentage, total]);
}

/* ============================================================================
 * UI COMPONENTS
 * ========================================================================== */
interface ProfileStrengthProps {
  profile: ProfileData | null | undefined;
  onPress?: () => void;
  showDetails?: boolean;
}

export function ProfileStrength({ 
  profile, 
  onPress,
  showDetails = false 
}: ProfileStrengthProps) {
  const { percentage, items, nextAction } = computeProfileStrength(profile);
  const dots = useProfileStrengthDots(percentage);
  const strengthLevel = getStrengthLevel(percentage);
  
  const completedItems = items.filter(item => item.completed);
  const incompleteItems = items.filter(item => !item.completed);

  const CardWrapper = onPress ? TouchableOpacity : View;

  return (
    <CardWrapper 
      style={[
        styles.strengthCard,
        { backgroundColor: strengthLevel.bgColor }
      ]} 
      onPress={onPress}
      activeOpacity={onPress ? 0.8 : 1}
    >
      <View style={styles.strengthContent}>
        <View style={styles.strengthLeft}>
          <Text style={[styles.strengthNumber, { color: strengthLevel.color }]}>
            {percentage}%
          </Text>
          <Text style={styles.strengthLabel}>Profile Complete</Text>
          <Text style={[styles.strengthLevel, { color: strengthLevel.color }]}>
            {strengthLevel.level}
          </Text>
        </View>
        
        <View style={styles.strengthRight}>
          <View style={styles.strengthDots}>
            {dots.map((filled, i) => (
              <View
                key={i}
                style={[
                  styles.strengthDot,
                  filled && [styles.strengthDotFilled, { backgroundColor: strengthLevel.color }]
                ]}
              />
            ))}
          </View>
          <Text style={styles.progressText}>
            {completedItems.length} of {items.length} completed
          </Text>
        </View>
      </View>

      {nextAction && !showDetails && (
        <View style={styles.nextActionBanner}>
          <View style={styles.nextActionHeader}>
            <Text style={styles.nextActionTitle}>Next Step</Text>
            {nextAction.priority === 'high' && (
              <View style={styles.highPriorityBadge}>
                <Text style={styles.highPriorityText}>RECOMMENDED</Text>
              </View>
            )}
          </View>
          <Text style={styles.nextActionText}>{nextAction.label}</Text>
        </View>
      )}

      {showDetails && (
        <View style={styles.detailsSection}>
          <Text style={styles.detailsTitle}>Profile Sections</Text>
          
          {completedItems.length > 0 && (
            <View style={styles.itemsGroup}>
              <Text style={styles.groupTitle}>✓ Completed</Text>
              {completedItems.map(item => (
                <View key={item.id} style={styles.itemRow}>
                  <View style={[styles.itemDot, styles.itemDotCompleted]} />
                  <Text style={styles.itemTextCompleted}>{item.label}</Text>
                </View>
              ))}
            </View>
          )}

          {incompleteItems.length > 0 && (
            <View style={styles.itemsGroup}>
              <Text style={styles.groupTitle}>○ To Complete</Text>
              {incompleteItems.map(item => (
                <View key={item.id} style={styles.itemRow}>
                  <View style={[styles.itemDot, styles.itemDotIncomplete]} />
                  <Text style={[
                    styles.itemTextIncomplete,
                    item.id === nextAction?.id && styles.itemTextNext
                  ]}>
                    {item.label}
                  </Text>
                  {item.id === nextAction?.id && (
                    <Text style={styles.nextBadge}>NEXT</Text>
                  )}
                  {item.priority === 'high' && item.id !== nextAction?.id && (
                    <Text style={styles.priorityBadge}>Important</Text>
                  )}
                </View>
              ))}
            </View>
          )}
        </View>
      )}

      {onPress && (
        <View style={styles.tapHint}>
          <Text style={styles.tapHintText}>Tap to improve →</Text>
        </View>
      )}
    </CardWrapper>
  );
}

/* ============================================================================
 * STYLES
 * ========================================================================== */
const styles = StyleSheet.create({
  strengthCard: {
    borderRadius: 20,
    padding: 20,
    marginTop: 16,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 8 },
    shadowOpacity: 0.2,
    shadowRadius: 16,
    elevation: 8,
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.1)",
  },
  strengthContent: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "flex-start",
  },
  strengthLeft: { 
    flex: 1,
    marginRight: 20,
  },
  strengthRight: {
    alignItems: "flex-end",
  },
  strengthNumber: {
    fontSize: 42,
    fontWeight: "900",
    lineHeight: 46,
    letterSpacing: -1,
  },
  strengthLabel: {
    color: "rgba(255,255,255,0.8)",
    fontSize: 14,
    fontWeight: "600",
    marginTop: 4,
    letterSpacing: 0.5,
  },
  strengthLevel: {
    fontSize: 12,
    fontWeight: "700",
    marginTop: 2,
    textTransform: "uppercase",
    letterSpacing: 1,
  },
  strengthDots: { 
    flexDirection: "row", 
    gap: 6,
    marginBottom: 8,
  },
  strengthDot: {
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: "rgba(255,255,255,0.2)",
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.1)",
  },
  strengthDotFilled: { 
    borderColor: "transparent",
    transform: [{ scale: 1.1 }],
  },
  progressText: {
    color: "rgba(255,255,255,0.6)",
    fontSize: 11,
    fontWeight: "500",
    textAlign: "right",
  },
  nextActionBanner: {
    marginTop: 16,
    padding: 16,
    backgroundColor: "rgba(255,255,255,0.08)",
    borderRadius: 12,
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.15)",
  },
  nextActionHeader: {
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "space-between",
    marginBottom: 6,
  },
  nextActionTitle: {
    color: "#FFFFFF",
    fontSize: 13,
    fontWeight: "700",
    letterSpacing: 0.5,
  },
  nextActionText: {
    color: "rgba(255,255,255,0.85)",
    fontSize: 15,
    fontWeight: "600",
    marginTop: 4,
  },
  highPriorityBadge: {
    backgroundColor: "rgba(239,68,68,0.25)",
    paddingHorizontal: 8,
    paddingVertical: 3,
    borderRadius: 6,
    borderWidth: 1,
    borderColor: "rgba(239,68,68,0.4)",
  },
  highPriorityText: {
    color: "#EF4444",
    fontSize: 9,
    fontWeight: "700",
    letterSpacing: 0.8,
  },
  detailsSection: {
    marginTop: 24,
    paddingTop: 20,
    borderTopWidth: 1,
    borderTopColor: "rgba(255,255,255,0.1)",
  },
  detailsTitle: {
    color: "#FFFFFF",
    fontSize: 16,
    fontWeight: "700",
    marginBottom: 16,
    letterSpacing: 0.5,
  },
  itemsGroup: {
    marginBottom: 16,
  },
  groupTitle: {
    color: "rgba(255,255,255,0.8)",
    fontSize: 13,
    fontWeight: "600",
    marginBottom: 12,
    textTransform: "uppercase",
    letterSpacing: 1,
  },
  itemRow: {
    flexDirection: "row",
    alignItems: "center",
    paddingVertical: 6,
    paddingHorizontal: 4,
  },
  itemDot: {
    width: 8,
    height: 8,
    borderRadius: 4,
    marginRight: 12,
  },
  itemDotCompleted: {
    backgroundColor: "#10B981",
  },
  itemDotIncomplete: {
    backgroundColor: "rgba(255,255,255,0.3)",
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.2)",
  },
  itemTextCompleted: {
    color: "rgba(255,255,255,0.9)",
    fontSize: 14,
    fontWeight: "500",
    flex: 1,
  },
  itemTextIncomplete: {
    color: "rgba(255,255,255,0.7)",
    fontSize: 14,
    fontWeight: "500",
    flex: 1,
  },
  itemTextNext: {
    color: "#FFFFFF",
    fontWeight: "600",
  },
  nextBadge: {
    backgroundColor: "rgba(59,130,246,0.25)",
    color: "#3B82F6",
    fontSize: 10,
    fontWeight: "700",
    paddingHorizontal: 8,
    paddingVertical: 2,
    borderRadius: 8,
    textTransform: "uppercase",
    letterSpacing: 0.5,
    borderWidth: 1,
    borderColor: "rgba(59,130,246,0.4)",
  },
  priorityBadge: {
    backgroundColor: "rgba(239,68,68,0.2)",
    color: "#EF4444",
    fontSize: 10,
    fontWeight: "600",
    paddingHorizontal: 8,
    paddingVertical: 2,
    borderRadius: 8,
    textTransform: "uppercase",
    letterSpacing: 0.5,
  },
  tapHint: {
    marginTop: 16,
    alignItems: "center",
  },
  tapHintText: {
    color: "rgba(255,255,255,0.6)",
    fontSize: 12,
    fontWeight: "500",
    letterSpacing: 0.5,
  },
});