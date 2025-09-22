import React, { useState } from "react";
import {
  View,
  Text,
  TextInput,
  TouchableOpacity,
  ScrollView,
  StyleSheet,
  Image,
  Dimensions,
  StatusBar,
} from "react-native";
import { Ionicons, Feather, MaterialIcons } from "@expo/vector-icons";
import { useNavigation } from '@react-navigation/native';

const { width } = Dimensions.get('window');

const DiscoverScreen = () => {
  const [searchQuery, setSearchQuery] = useState("");
  const [likedProfiles, setLikedProfiles] = useState(new Set());

  const toggleLike = (profileId) => {
    setLikedProfiles(prev => {
      const newSet = new Set(prev);
      if (newSet.has(profileId)) {
        newSet.delete(profileId);
      } else {
        newSet.add(profileId);
      }
      return newSet;
    });
  };

  const matches = [
    {
      id: 1,
      name: "Jordan",
      age: 22,
      pronouns: "He/Him",
      budget: "$800-1,200/mo",
      moveIn: "Next month",
      location: "1.8 miles away",
      activeTime: "Active 3 minutes ago",
      match: "92%",
      tags: ["STUDENT", "NIGHT OWL", "SOCIAL"],
      additionalTags: ["GYM ENTHUSIAST"],
      bio: "Junior at Northwestern studying marketing. Looking for someone chill who's down for occasional hangouts.",
      image: require("@/assets/blank-profile.png"),
      status: "Online",
      verified: true,
      verificationLevel: "bronze", // Added verification level
    },
    {
      id: 2,
      name: "Sam",
      age: 26,
      pronouns: "She/Her",
      budget: "$1,000-1,400/mo",
      moveIn: "Feb 1st",
      location: "3.4 miles away",
      activeTime: "Active 2 hours ago",
      match: "88%",
      tags: ["WORKING PROFESSIONAL", "PLANT PARENT"],
      additionalTags: ["MINIMALIST", "COFFEE LOVER"],
      bio: "Software engineer who works remotely. Love plants, minimalism, and good coffee. Looking for a clean, quiet space.",
      image: require("@/assets/blank-profile.png"),
      status: "Available",
      verified: true,
      verificationLevel: "platinum", // Added verification level
    },
    {
      id: 3,
      name: "Alex",
      age: 24,
      pronouns: "They/Them",
      budget: "$900-1,300/mo",
      moveIn: "March 15th",
      location: "2.2 miles away",
      activeTime: "Active 1 hour ago",
      match: "85%",
      tags: ["GRADUATE", "NON-SMOKER"],
      additionalTags: ["PET-FRIENDLY", "CLEAN"],
      bio: "PhD student in psychology. Love quiet evenings, books, and weekend adventures. Pet-friendly and super clean!",
      image: require("@/assets/blank-profile.png"),
      status: "Available",
      verified: true,
      verificationLevel: "silver", // Added verification level
    },
  ];

  const getTagStyle = (tag) => {
    const primaryTags = ["STUDENT", "WORKING PROFESSIONAL", "GRADUATE"];
    const greenTags = ["NON-SMOKER", "CLEAN"];
    const blueTags = ["GRADUATE"];
    
    if (primaryTags.includes(tag)) return styles.primaryTag;
    if (greenTags.includes(tag)) return styles.greenTag;
    if (blueTags.includes(tag)) return styles.blueTag;
    return styles.slateTag;
  };

  const getTagTextStyle = (tag) => {
    const primaryTags = ["STUDENT", "WORKING PROFESSIONAL", "GRADUATE"];
    const greenTags = ["NON-SMOKER", "CLEAN"];
    const blueTags = ["GRADUATE"];
    
    if (primaryTags.includes(tag)) return styles.primaryTagText;
    if (greenTags.includes(tag)) return styles.greenTagText;
    if (blueTags.includes(tag)) return styles.blueTagText;
    return styles.slateTagText;
  };

  const getStatusBadgeStyle = (status) => {
    if (status === "Online") return styles.onlineStatusBadge;
    if (status === "Available") return styles.availableStatusBadge;
    return styles.statusBadge;
  };

  // Function to get verification badge styling
  const getVerificationBadgeStyle = (level) => {
    const baseStyle = {
      position: "absolute",
      bottom: 16,
      left: 16,
      paddingHorizontal: 12,
      paddingVertical: 6,
      borderRadius: 16,
      flexDirection: "row",
      alignItems: "center",
      gap: 6,
      borderWidth: 1.5,
      borderColor: "#FFFFFF",
      shadowColor: "#000",
      shadowOffset: { width: 0, height: 2 },
      shadowOpacity: 0.1,
      shadowRadius: 6,
      elevation: 4,
    };
  
    switch (level) {
      case "bronze":
        return {
          ...baseStyle,
          backgroundColor: "#B87333", // Richer bronze
          borderColor: "#8B4513",
          shadowColor: "#B87333",
        };
      case "silver":
        return {
          ...baseStyle,
          backgroundColor: "#c0c0c0", // solid silver
        };
      case "gold":
        return {
          ...baseStyle,
          backgroundColor: "#FFD700", // Bright gold
        borderColor: "#DAA520",
        shadowColor: "#FFD700",
        };
      case "platinum":
        return {
          ...baseStyle,
          backgroundColor: "#f9f9f9", // platinum white
          borderColor: "#d4d4d8",
        };
      default:
        return {
          ...baseStyle,
          backgroundColor: "#9ca3af", // slate fallback
        };
    }
  };
  

  const getVerificationIcon = (level) => {
    switch (level) {
      case "bronze":
        return "medal-outline";
      case "silver":
        return "star-outline";
      case "gold":
        return "trophy-outline";
      case "platinum":
        return "diamond-outline";
      default:
        return "checkmark-circle";
    }
  };
  

  const getVerificationTextColor = (level) => {
    switch (level) {
      case "bronze":
        return "#FFFFFF";
      case "silver":
        return "#1F2937"; // dark gray text
      case "gold":
        return "#000000"; // black text
      case "platinum":
        return "#000000"; // black text for light bg
      default:
        return "#FFFFFF";
    }
  };
  
  const navigation = useNavigation();

  return (
    <View style={styles.container}>
      <StatusBar barStyle="light-content" backgroundColor="#1F2937" />
      
      {/* Header */}
      <View style={styles.header}>
        <View style={styles.headerContent}>
          <Text style={styles.title}>Discover</Text>
          <Text style={styles.subtitle}>Find your perfect roommate</Text>
        </View>
        <TouchableOpacity style={styles.filterIconButton}>
          <Feather name="sliders" size={20} color="#F8FAFC" />
        </TouchableOpacity>
      </View>

      {/* Search & Filter Section */}
      <View style={styles.searchSection}>
        <View style={styles.searchInputContainer}>
          <Ionicons name="search" size={18} color="#6B7280" style={styles.searchIcon} />
          <TextInput
            placeholder="Search profiles..."
            placeholderTextColor="#6B7280"
            style={styles.searchInput}
            value={searchQuery}
            onChangeText={setSearchQuery}
          />
          {searchQuery.length > 0 && (
            <TouchableOpacity 
              onPress={() => setSearchQuery("")}
              style={styles.clearButton}
            >
              <Ionicons name="close-circle" size={18} color="#6B7280" />
            </TouchableOpacity>
          )}
        </View>
        <TouchableOpacity style={styles.filterBtn}>
          <MaterialIcons name="tune" size={16} color="#F8FAFC" />
          <Text style={styles.filterText}>Filter</Text>
        </TouchableOpacity>
      </View>

      {/* Results Counter */}
      <View style={styles.resultsSection}>
        <Text style={styles.resultsText}>{matches.length} matches found</Text>
        <TouchableOpacity style={styles.sortButton}>
          <Text style={styles.sortText}>Best Match</Text>
          <Ionicons name="chevron-down" size={16} color="#6B7280" />
        </TouchableOpacity>
      </View>

      {/* Match Cards */}
      <ScrollView 
        style={styles.scrollContainer}
        showsVerticalScrollIndicator={false}
        contentContainerStyle={styles.scrollContent}
      >
        {matches.map((person) => (
          <View key={person.id} style={styles.card}>
            {/* Profile Image with Overlays */}
            <View style={styles.imageContainer}>
              <Image
                source={person.image}
                style={styles.profileImg}
              />
              
              {/* Gradient overlay for better text readability */}
              <View style={styles.imageOverlay} />
              
              {/* Match Badge */}
              <View style={styles.matchBadge}>
                <Ionicons name="heart" size={12} color="#FFFFFF" />
                <Text style={styles.matchText}>{person.match}</Text>
              </View>
              
              {/* Status Badge */}
              <View style={getStatusBadgeStyle(person.status)}>
                <View style={person.status === "Online" ? styles.onlineDot : styles.availableDot} />
                <Text style={styles.statusText}>{person.status}</Text>
              </View>

              {/* Verification Level Badge */}
              {person.verified && person.verificationLevel && (
                <View style={getVerificationBadgeStyle(person.verificationLevel)}>
                  <Ionicons 
                    name={getVerificationIcon(person.verificationLevel)} 
                    size={14} 
                    color={getVerificationTextColor(person.verificationLevel)} 
                  />
                  <Text style={[styles.verificationText, { color: getVerificationTextColor(person.verificationLevel) }]}>
                    {person.verificationLevel.toUpperCase()}
                  </Text>
                </View>
              )}

              {/* Verified Badge (kept the original blue checkmark) */}

            </View>

            {/* Card Content */}
            <View style={styles.cardContent}>
              {/* Name and Heart */}
              <View style={styles.nameRow}>
                <View style={styles.nameContainer}>
                  <Text style={styles.name}>
                    {person.name}, {person.age}
                  </Text>
                  <Text style={styles.pronouns}>{person.pronouns}</Text>
                </View>
                <TouchableOpacity 
                  style={styles.heartIcon}
                  onPress={() => toggleLike(person.id)}
                >
                  <Ionicons 
                    name={likedProfiles.has(person.id) ? "heart" : "heart-outline"} 
                    size={22} 
                    color={likedProfiles.has(person.id) ? "#EF4444" : "#6B7280"} 
                  />
                </TouchableOpacity>
              </View>

              {/* Location and Active Time */}
              <View style={styles.locationRow}>
                <View style={styles.locationItem}>
                  <Ionicons name="location" size={14} color="#EF4444" />
                  <Text style={styles.locationText}>{person.location}</Text>
                </View>
                <View style={styles.activeItem}>
                  <View style={styles.activeIndicator} />
                  <Text style={styles.activeText}>{person.activeTime}</Text>
                </View>
              </View>

              {/* Budget and Move-in Tags */}
              <View style={styles.mainTags}>
                <View style={styles.budgetTag}>
                <Text style={styles.tagText}>💵 </Text>
                  <View style={styles.budgetInfo}>
                    <Text style={styles.budgetLabel}>BUDGET</Text>
                    <Text style={styles.budgetValue}>{person.budget}</Text>
                  </View>
                </View>
                <View style={styles.moveInTag}>
                  <View style={styles.moveInIconContainer}>
                  <Text style={styles.tagText}>📦 </Text>
                  </View>
                  <View style={styles.moveInInfo}>
                    <Text style={styles.moveInLabel}>MOVE-IN</Text>
                    <Text style={styles.moveInValue}>{person.moveIn}</Text>
                  </View>
                </View>
              </View>

              {/* Personality Tags */}
              <View style={styles.tagContainer}>
                {person.tags.map((tag) => (
                  <View key={tag} style={getTagStyle(tag)}>
                    <Text style={getTagTextStyle(tag)}>{tag}</Text>
                  </View>
                ))}
              </View>

              {/* Additional Tags */}
              <View style={styles.tagContainer}>
                {person.additionalTags.map((tag) => (
                  <View key={tag} style={getTagStyle(tag)}>
                    <Text style={getTagTextStyle(tag)}>{tag}</Text>
                  </View>
                ))}
              </View>

              {/* Bio */}
              <Text style={styles.bio}>{person.bio}</Text>

              {/* Action Buttons */}
              <View style={styles.actionButtons}>
                <TouchableOpacity style={styles.messageBtn}>
                  <MaterialIcons name="message" size={16} color="#3B82F6" />
                  <Text style={styles.messageBtnText}>Message</Text>
                </TouchableOpacity>
                <TouchableOpacity style={styles.viewBtn} onPress={() => navigation.navigate('ProfilePreview')}>
                  <Text style={styles.viewBtnText}>View Profile</Text>
                  <Ionicons name="arrow-forward" size={16} color="#FFFFFF" />
                </TouchableOpacity>
              </View>
            </View>
          </View>
        ))}
      </ScrollView>
    </View>
  );
};

const styles = StyleSheet.create({
  container: {
    backgroundColor: "#0F172A",
    flex: 1,
  },
  header: {
    backgroundColor: "#1F2937",
    paddingTop: 50,
    paddingBottom: 20,
    paddingHorizontal: 20,
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 5,
  },
  headerContent: {
    flex: 1,
  },
  title: {
    color: "#F8FAFC",
    fontSize: 28,
    fontWeight: "700",
    marginBottom: 4,
  },
  subtitle: {
    color: "#E5E7EB",
    fontSize: 14,
    opacity: 0.8,
  },
  filterIconButton: {
    padding: 8,
    backgroundColor: "#374151",
    borderRadius: 8,
  },
  searchSection: {
    flexDirection: "row",
    gap: 12,
    marginHorizontal: 20,
    marginTop: 20,
    marginBottom: 16,
  },
  searchInputContainer: {
    flex: 1,
    flexDirection: "row",
    alignItems: "center",
    backgroundColor: "#FFFFFF",
    borderRadius: 12,
    paddingHorizontal: 16,
    borderWidth: 1,
    borderColor: "#E5E7EB",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 1 },
    shadowOpacity: 0.05,
    shadowRadius: 2,
    elevation: 2,
  },
  searchIcon: {
    marginRight: 12,
  },
  searchInput: {
    flex: 1,
    paddingVertical: 14,
    color: "#1F2937",
    fontSize: 16,
  },
  clearButton: {
    padding: 4,
  },
  filterBtn: {
    backgroundColor: "#3B82F6",
    paddingHorizontal: 18,
    paddingVertical: 14,
    borderRadius: 12,
    flexDirection: "row",
    alignItems: "center",
    gap: 6,
    shadowColor: "#3B82F6",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 4,
    elevation: 3,
  },
  filterText: {
    color: "#F8FAFC",
    fontWeight: "600",
    fontSize: 14,
  },
  resultsSection: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "center",
    marginHorizontal: 20,
    marginBottom: 16,
  },
  resultsText: {
    color: "#6B7280",
    fontSize: 14,
    fontWeight: "500",
  },
  sortButton: {
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
    paddingVertical: 4,
    paddingHorizontal: 8,
  },
  sortText: {
    color: "#6B7280",
    fontSize: 14,
    fontWeight: "500",
  },
  scrollContainer: {
    flex: 1,
  },
  scrollContent: {
    paddingHorizontal: 20,
    paddingBottom: 20,
  },
  card: {
    backgroundColor: "#FFFFFF",
    borderRadius: 20,
    marginBottom: 20,
    overflow: "hidden",
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 4 },
    shadowOpacity: 0.1,
    shadowRadius: 12,
    elevation: 6,
  },
  imageContainer: {
    position: "relative",
    height: 220,
  },
  profileImg: {
    width: "100%",
    height: "100%",
    resizeMode: "cover",
  },
  imageOverlay: {
    position: "absolute",
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: "rgba(0, 0, 0, 0.1)",
  },
  matchBadge: {
    position: "absolute",
    top: 16,
    right: 16,
    backgroundColor: "#10B981",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 4,
    elevation: 3,
  },
  matchText: {
    color: "#FFFFFF",
    fontSize: 12,
    fontWeight: "700",
  },
  onlineStatusBadge: {
    position: "absolute",
    top: 16,
    left: 16,
    backgroundColor: "rgba(34, 197, 94, 0.9)",
    paddingHorizontal: 10,
    paddingVertical: 6,
    borderRadius: 16,
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
  },
  availableStatusBadge: {
    position: "absolute",
    top: 16,
    left: 16,
    backgroundColor: "rgba(59, 130, 246, 0.9)",
    paddingHorizontal: 10,
    paddingVertical: 6,
    borderRadius: 16,
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
  },
  statusBadge: {
    position: "absolute",
    top: 16,
    left: 16,
    backgroundColor: "rgba(0, 0, 0, 0.6)",
    paddingHorizontal: 10,
    paddingVertical: 6,
    borderRadius: 16,
    flexDirection: "row",
    alignItems: "center",
    gap: 4,
  },
  onlineDot: {
    width: 6,
    height: 6,
    borderRadius: 3,
    backgroundColor: "#FFFFFF",
  },
  availableDot: {
    width: 6,
    height: 6,
    borderRadius: 3,
    backgroundColor: "#FFFFFF",
  },
  statusText: {
    color: "#FFFFFF",
    fontSize: 12,
    fontWeight: "600",
  },
  verifiedBadge: {
    position: "absolute",
    bottom: 16,
    right: 16,
    backgroundColor: "#FFFFFF",
    padding: 6,
    borderRadius: 20,
    shadowColor: "#000",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.1,
    shadowRadius: 4,
    elevation: 3,
  },
  // New styles for verification levels
  verificationText: {
    fontSize: 12,
    fontWeight: "700",
    letterSpacing: 0.5,
  },
  cardContent: {
    padding: 20,
  },
  nameRow: {
    flexDirection: "row",
    justifyContent: "space-between",
    alignItems: "flex-start",
    marginBottom: 12,
  },
  nameContainer: {
    flex: 1,
  },
  name: {
    color: "#1F2937",
    fontSize: 20,
    fontWeight: "700",
    marginBottom: 2,
  },
  pronouns: {
    color: "#6B7280",
    fontSize: 14,
    fontWeight: "500",
  },
  heartIcon: {
    padding: 8,
    marginTop: -4,
  },
  locationRow: {
    marginBottom: 20,
  },
  locationItem: {
    flexDirection: "row",
    alignItems: "center",
    marginBottom: 6,
  },
  locationText: {
    color: "#EF4444",
    fontSize: 14,
    marginLeft: 6,
    fontWeight: "500",
  },
  activeItem: {
    flexDirection: "row",
    alignItems: "center",
  },
  activeIndicator: {
    width: 6,
    height: 6,
    borderRadius: 3,
    backgroundColor: "#22C55E",
    marginRight: 8,
  },
  activeText: {
    color: "#6B7280",
    fontSize: 14,
    fontWeight: "500",
  },
  mainTags: {
    flexDirection: "row",
    gap: 12,
    marginBottom: 16,
  },
  budgetTag: {
    flex: 1,
    backgroundColor: "#F0FDF4",
    padding: 16,
    borderRadius: 12,
    flexDirection: "row",
    alignItems: "center",
    borderWidth: 1,
    borderColor: "#BBF7D0",
  },
  budgetIconContainer: {
    marginRight: 12,
  },
  budgetInfo: {
    flex: 1,
  },
  budgetLabel: {
    color: "#16A34A",
    fontSize: 11,
    fontWeight: "700",
    marginBottom: 2,
    letterSpacing: 0.5,
  },
  budgetValue: {
    color: "#22C55E",
    fontSize: 14,
    fontWeight: "700",
  },
  moveInTag: {
    flex: 1,
    backgroundColor: "#EFF6FF",
    padding: 16,
    borderRadius: 12,
    flexDirection: "row",
    alignItems: "center",
    borderWidth: 1,
    borderColor: "#DBEAFE",
  },
  moveInIconContainer: {
    marginRight: 12,
  },
  moveInInfo: {
    flex: 1,
  },
  moveInLabel: {
    color: "#1D4ED8",
    fontSize: 11,
    fontWeight: "700",
    marginBottom: 2,
    letterSpacing: 0.5,
  },
  moveInValue: {
    color: "#3B82F6",
    fontSize: 14,
    fontWeight: "700",
  },
  tagContainer: {
    flexDirection: "row",
    flexWrap: "wrap",
    gap: 8,
    marginBottom: 12,
  },
  primaryTag: {
    backgroundColor: "#3B82F6",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
  },
  primaryTagText: {
    color: "#FFFFFF",
    fontSize: 12,
    fontWeight: "600",
  },
  greenTag: {
    backgroundColor: "#10B981",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
  },
  greenTagText: {
    color: "#FFFFFF",
    fontSize: 12,
    fontWeight: "600",
  },
  blueTag: {
    backgroundColor: "#3B82F6",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
  },
  blueTagText: {
    color: "#FFFFFF",
    fontSize: 12,
    fontWeight: "600",
  },
  slateTag: {
    backgroundColor: "#64748B",
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
  },
  slateTagText: {
    color: "#FFFFFF",
    fontSize: 12,
    fontWeight: "600",
  },
  bio: {
    color: "#4B5563",
    fontSize: 15,
    lineHeight: 22,
    marginBottom: 20,
  },
  actionButtons: {
    flexDirection: "row",
    gap: 12,
  },
  messageBtn: {
    flex: 1,
    backgroundColor: "#FFFFFF",
    borderWidth: 1,
    borderColor: "#3B82F6",
    paddingVertical: 12,
    borderRadius: 12,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 6,
  },
  messageBtnText: {
    color: "#3B82F6",
    fontWeight: "600",
    fontSize: 14,
  },
  viewBtn: {
    flex: 1,
    backgroundColor: "#3B82F6",
    paddingVertical: 12,
    borderRadius: 12,
    flexDirection: "row",
    alignItems: "center",
    justifyContent: "center",
    gap: 6,
    shadowColor: "#3B82F6",
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.2,
    shadowRadius: 4,
    elevation: 3,
  },
  viewBtnText: {
    color: "#FFFFFF",
    fontWeight: "600",
    fontSize: 14,
  },
});

export default DiscoverScreen;