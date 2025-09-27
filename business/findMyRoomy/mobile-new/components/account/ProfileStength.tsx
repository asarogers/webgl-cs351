import React, { useMemo } from "react";
import { StyleSheet, Text, View, TouchableOpacity } from "react-native";

/* ============================================================================
 * TYPES & INTERFACES
 * ========================================================================== */
interface ProfileData {
  first_name?: string;
  last_name?: string;
  about?: string;
  photos?: string[];
  interests?: string[];
  budget_min?: number;
  budget_max?: number;
  education?: string;
  company?: string;
  pet_situation?: string;
  weekend_vibe?: string;
  sleep_schedule?: string;
  location_sharing?: boolean;
  zipcode?: string;
}

interface ProfileStrengthItem {
  id: string;
  label: string;
  completed: boolean;
  priority: 'high' | 'medium' | 'low';
}

/* ============================================================================
 * CALCULATION LOGIC
 * ========================================================================== */
export function computeProfileStrength(profile: ProfileData | null | undefined): {
  score: number;
  percentage: number;
  items: ProfileStrengthItem[];
} {
  if (!profile) {
    return { score: 0, percentage: 0, items: [] };
  }

  const items: ProfileStrengthItem[] = [
    {
      id: 'name',
      label: 'Full name',
      completed: Boolean(profile.first_name && profile.last_name),
      priority: 'high'
    },
    {
      id: 'photos',
      label: 'Profile photos',
      completed: Array.isArray(profile.photos) && profile.photos.length > 0,
      priority: 'high'
    },
    {
      id: 'about',
      label: 'About section (20+ characters)',
      completed: Boolean(profile.about && profile.about.trim().length > 20),
      priority: 'high'
    },
    {
      id: 'interests',
      label: 'Interests (3+ selected)',
      completed: Array.isArray(profile.interests) && profile.interests.length >= 3,
      priority: 'medium'
    },
    {
      id: 'budget',
      label: 'Budget information',
      completed: Boolean(profile.budget_min || profile.budget_max),
      priority: 'medium'
    },
    {
      id: 'work_education',
      label: 'Work or education',
      completed: Boolean(profile.education || profile.company),
      priority: 'medium'
    },
    {
      id: 'lifestyle',
      label: 'Lifestyle preferences',
      completed: Boolean(profile.pet_situation && profile.pet_situation !== "none"),
      priority: 'low'
    },
    {
      id: 'schedule',
      label: 'Schedule preferences',
      completed: Boolean(profile.weekend_vibe || profile.sleep_schedule),
      priority: 'low'
    },
    {
      id: 'location',
      label: 'Location details',
      completed: Boolean(profile.location_sharing && profile.zipcode),
      priority: 'low'
    }
  ];

  const completedCount = items.filter(item => item.completed).length;
  const percentage = Math.round((completedCount / items.length) * 100);

  return {
    score: completedCount,
    percentage,
    items
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
  const { percentage, items } = computeProfileStrength(profile);
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
                  <Text style={styles.itemTextIncomplete}>{item.label}</Text>
                  {item.priority === 'high' && (
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