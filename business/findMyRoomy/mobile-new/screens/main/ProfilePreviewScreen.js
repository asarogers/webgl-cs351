import React from "react";
import { View, Text, TouchableOpacity, ScrollView, StyleSheet, Dimensions } from "react-native";
import { LinearGradient } from 'expo-linear-gradient';
import { useNavigation } from '@react-navigation/native';

const { width } = Dimensions.get('window');

const profile = {
  name: "Sarah Johnson",
  badge: "Gold",
  details: ["24 years old • She/Her • Graduate Student", "0.3 miles away • Lincoln Park"],
  onlineStatus: "Active 2 hours ago",
  mainPhoto: "📸",
  photos: ["🏠", "🎓", "🌟", "🎵"],
  compatibility: {
    score: "89%",
    factors: [
      { icon: "🏠", label: "Lifestyle" },
      { icon: "💰", label: "Budget" },
      { icon: "📍", label: "Location" },
      { icon: "⏰", label: "Schedule" }
    ]
  },
  about: [
    "Hey! I'm a graduate student at Northwestern studying environmental science. I love keeping things clean and organized, but I'm also down for spontaneous adventures around the city. Looking for someone who appreciates good coffee, deep conversations, and the occasional Netflix binge.",
    "I work from home most days but love getting out to explore Chicago's food scene and parks. Pet-friendly and 420-friendly! Let's find our perfect place together."
  ],
  tags: [
    { text: "Non-smoker", color: "#10B981" },
    { text: "Pet-friendly", color: "#64748B" },
    { text: "Graduate Student", color: "#3B82F6" },
    { text: "Early Bird", color: "#64748B" },
    { text: "Clean", color: "#10B981" },
    { text: "Social", color: "#64748B" },
    { text: "Coffee Lover", color: "#64748B" },
    { text: "Foodie", color: "#64748B" }
  ],
  lifestyle: [
    { icon: "🌅", title: "Sleep Schedule", value: "Early Bird" },
    { icon: "🎉", title: "Social Level", value: "Moderately Social" },
    { icon: "🧼", title: "Cleanliness", value: "Very Clean" },
    { icon: "🎵", title: "Noise Level", value: "Quiet" },
    { icon: "🍳", title: "Cooking", value: "Love to Cook" },
    { icon: "🐕", title: "Pets", value: "Cat & Dog Friendly" }
  ],
  housing: [
    { title: "Budget Range", value: "$800-1000", detail: "per month" },
    { title: "Move-in Date", value: "Aug 1st", detail: "Flexible ±2 weeks" },
    { title: "Lease Length", value: "12 months", detail: "Preferred" },
    { title: "Preferred Areas", value: "3 zones", detail: "Lincoln Park, Lakeview, Logan Square" }
  ],
  mutual: {
    avatars: ["A", "M", "J", "K", "L"],
    count: 5
  },
  verifications: [
    "Phone number verified",
    "Email verified",
    "Government ID verified",
    "Income verified",
    "References checked"
  ]
};

export default function FullProfileScreen() {
    const navigation = useNavigation();
    // Height of header (adjust if you change style)
    const HEADER_HEIGHT = 260;
  
    return (
      <View style={styles.container}>
        {/* Absolutely positioned header */}
        <LinearGradient
          colors={['#1F2937', '#374151']}
          start={{ x: 0, y: 0 }}
          end={{ x: 1, y: 1 }}
          style={[styles.header, { height: HEADER_HEIGHT }]}
        >
          <View style={styles.headerNav}>
            <TouchableOpacity
              style={styles.backBtn}
              onPress={() => navigation.goBack()}
            >
              <Text style={styles.backBtnText}>←</Text>
            </TouchableOpacity>
            <View style={styles.headerActions}>
              <TouchableOpacity style={styles.headerBtn}>
                <Text style={styles.headerBtnText}>Share</Text>
              </TouchableOpacity>
              <TouchableOpacity style={styles.headerBtn}>
                <Text style={styles.headerBtnText}>Save</Text>
              </TouchableOpacity>
              <TouchableOpacity style={styles.headerBtn}>
                <Text style={styles.headerBtnText}>⋯</Text>
              </TouchableOpacity>
            </View>
          </View>
          <View style={styles.profileHeader}>
            <View style={styles.avatarMain}>
              <Text style={styles.avatarMainText}>S</Text>
            </View>
            <View style={styles.profileInfo}>
              <View style={styles.profileNameRow}>
                <Text style={styles.profileName}>{profile.name}</Text>
                <View style={styles.verificationBadge}>
                  <Text style={styles.verificationBadgeText}>{profile.badge}</Text>
                </View>
              </View>
              {profile.details.map((d, i) => (
                <Text key={i} style={styles.profileDetails}>{d}</Text>
              ))}
              <View style={styles.onlineStatus}>
                <View style={styles.onlineDot} />
                <Text style={styles.onlineStatusText}>{profile.onlineStatus}</Text>
              </View>
            </View>
          </View>
        </LinearGradient>

        <ScrollView
        showsVerticalScrollIndicator={false}
        bounces={true}
        style={{ flex: 1 }}
        contentContainerStyle={{ paddingTop: HEADER_HEIGHT + 8 }} // a little extra gap
      >
       

        {/* Main Content */}
        <View style={styles.mainContent}>
          {/* Photo Gallery - Enhanced */}
          <View style={styles.photoSection}>
            <Text style={styles.sectionTitle}>Photos</Text>
            <View style={styles.photoGallery}>
              <TouchableOpacity style={styles.photoMain}>
                <Text style={styles.photoMainIcon}>{profile.mainPhoto}</Text>
                <View style={styles.photoCount}>
                  <Text style={styles.photoCountText}>+{profile.photos.length} photos</Text>
                </View>
              </TouchableOpacity>
              <View style={styles.photoGrid}>
                {profile.photos.map((icon, i) => (
                  <TouchableOpacity key={i} style={styles.photoSmall}>
                    <Text style={styles.photoSmallIcon}>{icon}</Text>
                  </TouchableOpacity>
                ))}
              </View>
            </View>
          </View>

          {/* Compatibility Score - Enhanced */}
          <View style={styles.compatibilitySection}>
            <LinearGradient
              colors={['#3B82F6', '#60A5FA']}
              style={styles.compatibilityCard}
            >
              <Text style={styles.compatibilityScore}>{profile.compatibility.score}</Text>
              <Text style={styles.compatibilityLabel}>Compatibility Match</Text>
              <View style={styles.compatibilityFactors}>
                {profile.compatibility.factors.map((f, i) => (
                  <View style={styles.compatibilityFactor} key={i}>
                    <View style={styles.factorIconContainer}>
                      <Text style={styles.factorIcon}>{f.icon}</Text>
                    </View>
                    <Text style={styles.factorLabel}>{f.label}</Text>
                  </View>
                ))}
              </View>
            </LinearGradient>
          </View>

          {/* About Section - Enhanced */}
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>About Sarah</Text>
            <View style={styles.aboutCard}>
              {profile.about.map((text, i) => (
                <Text key={i} style={styles.aboutText}>{text}</Text>
              ))}
              <View style={styles.tagsContainer}>
                {profile.tags.map((tag, i) => (
                  <View style={[styles.tag, { backgroundColor: tag.color }]} key={i}>
                    <Text style={styles.tagText}>{tag.text}</Text>
                  </View>
                ))}
              </View>
            </View>
          </View>

          {/* Lifestyle Preferences - Enhanced */}
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>Lifestyle</Text>
            <View style={styles.lifestyleGrid}>
              {profile.lifestyle.map((item, i) => (
                <View style={styles.lifestyleCard} key={i}>
                  <View style={styles.lifestyleIconContainer}>
                    <Text style={styles.lifestyleIcon}>{item.icon}</Text>
                  </View>
                  <Text style={styles.lifestyleTitle}>{item.title}</Text>
                  <Text style={styles.lifestyleValue}>{item.value}</Text>
                </View>
              ))}
            </View>
          </View>

          {/* Budget & Location - Enhanced */}
          <View style={styles.section}>
            <Text style={styles.sectionTitle}>Housing Details</Text>
            <View style={styles.infoCards}>
              {profile.housing.map((item, i) => (
                <View style={styles.infoCard} key={i}>
                  <Text style={styles.infoCardTitle}>{item.title}</Text>
                  <Text style={[styles.infoCardValue, 
                    item.title === "Budget Range" ? styles.priceText : null
                  ]}>{item.value}</Text>
                  <Text style={styles.infoCardDetail}>{item.detail}</Text>
                </View>
              ))}
            </View>
          </View>

          {/* Mutual Connections - Enhanced */}
          <View style={styles.mutualSection}>
            <Text style={styles.mutualTitle}>Mutual Connections</Text>
            <View style={styles.mutualAvatars}>
              {profile.mutual.avatars.map((v, i) => (
                <View key={i} style={styles.mutualAvatar}>
                  <Text style={styles.mutualAvatarText}>{v}</Text>
                </View>
              ))}
            </View>
            <Text style={styles.mutualCount}>
              You have {profile.mutual.count} mutual connections
            </Text>
          </View>

          {/* Safety & Verification - Enhanced */}
          <View style={styles.safetySection}>
            <Text style={styles.safetyTitle}>Trust & Safety</Text>
            <View style={styles.verificationItems}>
              {profile.verifications.map((v, i) => (
                <View style={styles.verificationItem} key={i}>
                  <View style={styles.verificationIconContainer}>
                    <Text style={styles.verificationIcon}>✅</Text>
                  </View>
                  <Text style={styles.verificationText}>{v}</Text>
                </View>
              ))}
            </View>
          </View>

          {/* Bottom spacing for action buttons */}
          <View style={{ height: 100 }} />
        </View>
      </ScrollView>

      {/* Action Buttons - Enhanced */}
      <View style={styles.actionButtons}>
        <TouchableOpacity style={styles.btnSecondary}>
          <Text style={styles.btnSecondaryText}>💬 Message</Text>
        </TouchableOpacity>
        <TouchableOpacity style={styles.btnPrimary}>
          <LinearGradient
            colors={['#3B82F6', '#2563EB']}
            style={styles.btnPrimaryGradient}
          >
            <Text style={styles.btnPrimaryText}>✨ Connect</Text>
          </LinearGradient>
        </TouchableOpacity>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  container: { 
    flex: 1, 
    backgroundColor: "#F8FAFC" 
  },
  
  // Header styles - now scrollable
  header: {
    position: 'absolute',
    top: 0,              
    left: 0,             
    right: 0,          
    zIndex: 10,          
    paddingTop: 50,
    paddingBottom: 0,
    paddingHorizontal: 20,
    borderBottomLeftRadius: 24,
    borderBottomRightRadius: 24,
    marginTop: 0,
    elevation: 1,
  },
  
  headerNav: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginBottom: 20,
  },
  backBtn: {
    backgroundColor: "rgba(255,255,255,0.15)",
    borderRadius: 28,
    width: 48,
    height: 48,
    justifyContent: "center",
    alignItems: "center",
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.2)",
  },
  backBtnText: { 
    color: "#fff", 
    fontSize: 20,
    fontWeight: "bold"
  },
  headerActions: { 
    flexDirection: "row", 
    gap: 8 
  },
  headerBtn: {
    backgroundColor: "rgba(255,255,255,0.15)",
    borderRadius: 20,
    paddingHorizontal: 16,
    paddingVertical: 8,
    borderWidth: 1,
    borderColor: "rgba(255,255,255,0.2)",
  },
  headerBtnText: { 
    color: "#fff", 
    fontSize: 13,
    fontWeight: "600"
  },
  profileHeader: { 
    flexDirection: "row", 
    alignItems: "center", 
    gap: 16 
  },
  avatarMain: {
    width: 80, 
    height: 80, 
    borderRadius: 40,
    alignItems: "center", 
    justifyContent: "center",
    backgroundColor: "#3B82F6",
    borderWidth: 3, 
    borderColor: "rgba(255,255,255,0.3)",
  },
  avatarMainText: { 
    color: "#fff", 
    fontSize: 28, 
    fontWeight: "bold" 
  },
  profileInfo: {
    flex: 1
  },
  profileNameRow: { 
    flexDirection: "row", 
    alignItems: "center", 
    marginBottom: 4
  },
  profileName: { 
    color: "#fff", 
    fontSize: 26, 
    fontWeight: "bold" 
  },
  verificationBadge: {
    backgroundColor: "#3B82F6", 
    paddingHorizontal: 10, 
    paddingVertical: 3,
    borderRadius: 14, 
    marginLeft: 12,
  },
  verificationBadgeText: { 
    color: "#fff", 
    fontSize: 11, 
    fontWeight: "bold" 
  },
  profileDetails: { 
    color: "rgba(255,255,255,0.9)", 
    fontSize: 15,
    marginBottom: 2
  },
  onlineStatus: { 
    flexDirection: "row", 
    alignItems: "center", 
    marginTop: 8
  },
  onlineDot: {
    width: 10, 
    height: 10, 
    borderRadius: 5, 
    backgroundColor: "#22C55E", 
    marginRight: 8,
  },
  onlineStatusText: { 
    color: "rgba(255,255,255,0.8)", 
    fontSize: 13 
  },
  
  // Main content
  mainContent: { 
    paddingHorizontal: 20, 
    paddingVertical: 24 
  },
  section: { 
    marginBottom: 32 
  },
  sectionTitle: { 
    fontSize: 22, 
    fontWeight: "bold", 
    color: "#1F2937", 
    marginBottom: 16 
  },
  
  // Photo section
  photoSection: { 
    marginBottom: 32 
  },
  photoGallery: { 
    flexDirection: "row", 
    gap: 12 
  },
  photoMain: {
    backgroundColor: "#E5E7EB", 
    borderRadius: 20, 
    height: 200, 
    width: width * 0.45,
    justifyContent: "center", 
    alignItems: "center", 
    position: "relative",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.1,
    shadowRadius: 8,
    elevation: 3,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  photoMainIcon: { 
    fontSize: 56, 
    color: "#6B7280" 
  },
  photoCount: {
    position: "absolute", 
    bottom: 16, 
    right: 16,
    backgroundColor: "#111827", 
    borderRadius: 16, 
    paddingHorizontal: 12, 
    paddingVertical: 6,
  },
  photoCountText: { 
    color: "#fff", 
    fontSize: 12,
    fontWeight: "600"
  },
  photoGrid: {
    flex: 1,
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 8,
  },
  photoSmall: {
    backgroundColor: "#E5E7EB", 
    borderRadius: 16, 
    height: 92, 
    width: (width * 0.45 - 16) / 2,
    justifyContent: "center", 
    alignItems: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.08,
    shadowRadius: 4,
    elevation: 2,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  photoSmallIcon: { 
    fontSize: 28, 
    color: "#6B7280" 
  },
  
  // Compatibility section
  compatibilitySection: {
    marginBottom: 32,
  },
  compatibilityCard: {
    borderRadius: 24, 
    padding: 28, 
    alignItems: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 8 },
    shadowOpacity: 0.15,
    shadowRadius: 16,
    elevation: 5,
  },
  compatibilityScore: { 
    color: "#fff", 
    fontSize: 52, 
    fontWeight: "bold", 
    marginBottom: 8 
  },
  compatibilityLabel: { 
    color: "#fff", 
    fontSize: 18, 
    opacity: 0.9, 
    marginBottom: 20 
  },
  compatibilityFactors: { 
    flexDirection: "row", 
    justifyContent: "space-around", 
    width: "100%"
  },
  compatibilityFactor: { 
    alignItems: "center", 
    flex: 1 
  },
  factorIconContainer: {
    backgroundColor: "rgba(255,255,255,0.2)",
    borderRadius: 20,
    width: 40,
    height: 40,
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 8,
  },
  factorIcon: { 
    fontSize: 20, 
    color: "#fff" 
  },
  factorLabel: { 
    color: "#fff", 
    fontSize: 13, 
    opacity: 0.9,
    fontWeight: "500"
  },
  
  // About section
  aboutCard: {
    backgroundColor: "#fff",
    borderRadius: 20,
    padding: 24,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08,
    shadowRadius: 12,
    elevation: 3,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  aboutText: { 
    fontSize: 16, 
    lineHeight: 24, 
    color: "#374151", 
    marginBottom: 16 
  },
  tagsContainer: { 
    flexDirection: "row", 
    flexWrap: "wrap", 
    gap: 8,
    marginTop: 8
  },
  tag: {
    borderRadius: 20, 
    paddingHorizontal: 14, 
    paddingVertical: 8,
  },
  tagText: { 
    color: "#fff", 
    fontSize: 13, 
    fontWeight: "600" 
  },
  
  // Lifestyle section
  lifestyleGrid: { 
    flexDirection: "row", 
    flexWrap: "wrap", 
    gap: 12 
  },
  lifestyleCard: {
    backgroundColor: "#fff", 
    borderRadius: 20, 
    padding: 20,
    width: (width - 52) / 2,
    alignItems: "center",
    shadowColor: "#000", 
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08, 
    shadowRadius: 12, 
    elevation: 3,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  lifestyleIconContainer: {
    backgroundColor: "#F8FAFC",
    borderRadius: 20,
    width: 44,
    height: 44,
    justifyContent: "center",
    alignItems: "center",
    marginBottom: 12,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  lifestyleIcon: { 
    fontSize: 24 
  },
  lifestyleTitle: { 
    fontSize: 15, 
    fontWeight: "bold", 
    marginBottom: 4, 
    color: "#1F2937",
    textAlign: "center"
  },
  lifestyleValue: { 
    fontSize: 13, 
    color: "#6B7280",
    textAlign: "center"
  },
  
  // Info cards
  infoCards: { 
    flexDirection: "row", 
    flexWrap: "wrap", 
    gap: 12 
  },
  infoCard: {
    backgroundColor: "#fff", 
    borderRadius: 20, 
    padding: 20, 
    width: (width - 52) / 2,
    shadowColor: "#000", 
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08, 
    shadowRadius: 12, 
    elevation: 3,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  infoCardTitle: { 
    fontSize: 15, 
    fontWeight: "bold", 
    marginBottom: 8, 
    color: "#1F2937" 
  },
  infoCardValue: { 
    fontSize: 20, 
    fontWeight: "bold", 
    color: "#3B82F6", 
    marginBottom: 4 
  },
  priceText: {
    color: "#22C55E",
  },
  infoCardDetail: { 
    fontSize: 13, 
    color: "#6B7280" 
  },
  
  // Mutual section
  mutualSection: {
    backgroundColor: "#fff", 
    borderRadius: 20, 
    padding: 24, 
    marginBottom: 32,
    shadowColor: "#000", 
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08, 
    shadowRadius: 12, 
    elevation: 3,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  mutualTitle: { 
    fontSize: 18, 
    fontWeight: "bold", 
    marginBottom: 16, 
    color: "#1F2937" 
  },
  mutualAvatars: { 
    flexDirection: "row", 
    gap: 8, 
    marginBottom: 12 
  },
  mutualAvatar: {
    width: 36, 
    height: 36, 
    borderRadius: 18,
    alignItems: "center", 
    justifyContent: "center",
    backgroundColor: "#3B82F6",
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  mutualAvatarText: { 
    color: "#fff", 
    fontWeight: "bold",
    fontSize: 14
  },
  mutualCount: { 
    fontSize: 14, 
    color: "#6B7280" 
  },
  
  // Safety section
  safetySection: {
    backgroundColor: "#fff", 
    borderRadius: 20, 
    padding: 24, 
    marginBottom: 32,
    borderWidth: 1,
    borderColor: "#E5E7EB",
    shadowColor: "#000", 
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.08, 
    shadowRadius: 12, 
    elevation: 3,
  },
  safetyTitle: { 
    fontSize: 18, 
    fontWeight: "bold", 
    marginBottom: 16, 
    color: "#1F2937" 
  },
  verificationItems: { 
    gap: 8 
  },
  verificationItem: {
    flexDirection: "row", 
    alignItems: "center", 
    backgroundColor: "#F8FAFC", 
    borderRadius: 16, 
    paddingHorizontal: 16, 
    paddingVertical: 12,
    borderWidth: 1,
    borderColor: "#E5E7EB",
  },
  verificationIconContainer: {
    marginRight: 12,
  },
  verificationIcon: { 
    fontSize: 18, 
    color: "#10B981" 
  },
  verificationText: { 
    fontSize: 15, 
    color: "#1F2937",
    fontWeight: "500"
  },
  
  // Action buttons
  actionButtons: {
    position: 'absolute',
    bottom: 0,
    left: 0,
    right: 0,
    flexDirection: "row", 
    gap: 12, 
    padding: 20, 
    backgroundColor: "#fff",
    borderTopWidth: 1, 
    borderTopColor: "#E5E7EB",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: -4 },
    shadowOpacity: 0.1,
    shadowRadius: 8,
    elevation: 5,
  },
  btnSecondary: {
    backgroundColor: "#fff", 
    borderColor: "#3B82F6", 
    borderWidth: 2,
    borderRadius: 16, 
    paddingVertical: 16, 
    flex: 1, 
    alignItems: "center",
    justifyContent: "center",
  },
  btnSecondaryText: { 
    color: "#3B82F6", 
    fontSize: 16, 
    fontWeight: "bold" 
  },
  btnPrimary: {
    flex: 1,
    borderRadius: 16,
    overflow: "hidden",
  },
  btnPrimaryGradient: {
    paddingVertical: 16,
    alignItems: "center",
    justifyContent: "center",
  },
  btnPrimaryText: { 
    color: "#fff", 
    fontSize: 16, 
    fontWeight: "bold" 
  }
});