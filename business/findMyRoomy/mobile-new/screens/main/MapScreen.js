import React, { useState } from 'react';
import {
  View,
  Text,
  TextInput,
  TouchableOpacity,
  StyleSheet,
  ScrollView,
  Dimensions,
  Animated,
} from 'react-native';
import { SafeAreaView } from 'react-native-safe-area-context';
import { Ionicons, MaterialIcons } from '@expo/vector-icons';

const { width, height } = Dimensions.get('window');

const RoommateMapScreen = () => {
  const [selectedFilter, setSelectedFilter] = useState('All');
  const [favorites, setFavorites] = useState(new Set());
  const [showFilters, setShowFilters] = useState(false);

  const filters = ['All', 'Near Me', 'Budget', 'Verified', 'Online'];

  const mockRoommates = [
    {
      id: 1,
      name: 'Sarah Johnson',
      age: 24,
      distance: '0.3 miles',
      price: 850,
      bio: 'Graduate student looking for a clean, quiet roommate in downtown. Pet-friendly apartment.',
      tags: ['Non-smoker', 'Pet-friendly', 'Graduate'],
      online: true,
      verified: true,
      rating: 4.8,
      avatar: 'S'
    },
    {
      id: 2,
      name: 'Mike Chen',
      age: 26,
      distance: '0.7 miles',
      price: 950,
      bio: 'Software engineer seeking responsible roommate. Great location near tech hub.',
      tags: ['Professional', 'Gym-goer', 'Tech'],
      online: false,
      verified: true,
      rating: 4.9,
      avatar: 'M'
    },
    {
      id: 3,
      name: 'Emma Rodriguez',
      age: 23,
      distance: '1.2 miles',
      price: 750,
      bio: 'Art student looking for creative roommate. Spacious apartment with natural light.',
      tags: ['Creative', 'Student', 'Art'],
      online: true,
      verified: false,
      rating: 4.6,
      avatar: 'E'
    }
  ];

  const toggleFavorite = (id) => {
    const newFavorites = new Set(favorites);
    if (newFavorites.has(id)) {
      newFavorites.delete(id);
    } else {
      newFavorites.add(id);
    }
    setFavorites(newFavorites);
  };

  const getTagStyle = (tag) => {
    const tagStyles = {
      'Non-smoker': { backgroundColor: '#10B981', color: '#FFFFFF' },
      'Pet-friendly': { backgroundColor: '#64748B', color: '#FFFFFF' },
      'Graduate': { backgroundColor: '#3B82F6', color: '#FFFFFF' },
      'Professional': { backgroundColor: '#3B82F6', color: '#FFFFFF' },
      'Gym-goer': { backgroundColor: '#10B981', color: '#FFFFFF' },
      'Tech': { backgroundColor: '#3B82F6', color: '#FFFFFF' },
      'Creative': { backgroundColor: '#F59E0B', color: '#FFFFFF' },
      'Student': { backgroundColor: '#8B5CF6', color: '#FFFFFF' },
      'Art': { backgroundColor: '#F59E0B', color: '#FFFFFF' }
    };
    return tagStyles[tag] || { backgroundColor: '#64748B', color: '#FFFFFF' };
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      {/* Header */}
      <View style={styles.header}>
        <Text style={styles.logo}>MyRoomie</Text>
        <TouchableOpacity 
          style={styles.filterBtn}
          onPress={() => setShowFilters(!showFilters)}
        >
          <Ionicons name="filter" size={16} color="#FFFFFF" />
          <Text style={styles.filterText}>Filters</Text>
        </TouchableOpacity>
      </View>

      {/* Search Bar */}
      <View style={styles.searchRow}>
        <Ionicons name="search" size={20} color="#9CA3AF" />
        <TextInput
          placeholder="Search location, neighborhood..."
          placeholderTextColor="#9CA3AF"
          style={styles.searchInput}
        />
        <TouchableOpacity style={styles.mapToggle}>
          <Text style={styles.mapToggleText}>Map View</Text>
        </TouchableOpacity>
      </View>

      {/* Advanced Filters Panel */}
      {showFilters && (
        <View style={styles.advancedFilters}>
          <View style={styles.filterRow}>
            <View style={styles.filterGroup}>
              <Text style={styles.filterLabel}>Price Range</Text>
              <View style={styles.priceInputs}>
                <TextInput 
                  style={styles.priceInput} 
                  placeholder="Min" 
                  placeholderTextColor="#9CA3AF"
                />
                <Text style={styles.priceSeparator}>-</Text>
                <TextInput 
                  style={styles.priceInput} 
                  placeholder="Max" 
                  placeholderTextColor="#9CA3AF"
                />
              </View>
            </View>
            <View style={styles.filterGroup}>
              <Text style={styles.filterLabel}>Distance</Text>
              <TouchableOpacity style={styles.selectInput}>
                <Text style={styles.selectText}>Within 1 mile</Text>
                <Ionicons name="chevron-down" size={16} color="#9CA3AF" />
              </TouchableOpacity>
            </View>
          </View>
        </View>
      )}

      {/* Map Area */}
      <View style={styles.mapArea}>
        <View style={styles.mapPlaceholder}>
          <Ionicons name="map" size={48} color="#9CA3AF" />
          <Text style={styles.mapText}>Interactive Map View</Text>
          <Text style={styles.mapSubtext}>Showing 24 roommates nearby</Text>
        </View>
        
        {/* Sample Map Pins */}
        <View style={[styles.mapPin, { top: height * 0.2, left: width * 0.3 }]} />
        <View style={[styles.mapPin, { top: height * 0.35, right: width * 0.25 }]} />
        <View style={[styles.mapPin, { bottom: height * 0.25, left: width * 0.5 }]} />
        
        {/* Zoom Controls */}
        <View style={styles.zoomControls}>
          <TouchableOpacity style={styles.zoomBtn}>
            <Ionicons name="add" size={24} color="#FFFFFF" />
          </TouchableOpacity>
          <TouchableOpacity style={styles.zoomBtn}>
            <Ionicons name="remove" size={24} color="#FFFFFF" />
          </TouchableOpacity>
        </View>
      </View>

      {/* Bottom Panel */}
      <View style={styles.bottomPanel}>
        {/* Quick Filters */}
        <View style={styles.filtersSection}>
          <ScrollView horizontal showsHorizontalScrollIndicator={false} style={styles.filtersRow}>
            {filters.map((filter) => (
              <TouchableOpacity
                key={filter}
                style={[
                  styles.filterChip,
                  selectedFilter === filter && styles.filterChipActive
                ]}
                onPress={() => setSelectedFilter(filter)}
              >
                <Text style={[
                  styles.filterChipText,
                  selectedFilter === filter && styles.filterChipTextActive
                ]}>
                  {filter}
                </Text>
              </TouchableOpacity>
            ))}
          </ScrollView>
          
          {/* Results & Sort */}
          <View style={styles.resultRow}>
            <Text style={styles.resultCount}>24 results nearby</Text>
            <TouchableOpacity style={styles.sortButton}>
              <Text style={styles.sortText}>Distance</Text>
              <Ionicons name="chevron-down" size={16} color="#3B82F6" />
            </TouchableOpacity>
          </View>
        </View>

        {/* Roommate Cards */}
        <ScrollView style={styles.cardsList} showsVerticalScrollIndicator={false}>
          {mockRoommates.map((roommate) => (
            <View key={roommate.id} style={styles.card}>
              {/* Card Header */}
              <View style={styles.cardHeader}>
                <View style={styles.avatar}>
                  <Text style={styles.avatarText}>{roommate.avatar}</Text>
                </View>
                <View style={styles.userInfo}>
                  <View style={styles.nameRow}>
                    <Text style={styles.name}>{roommate.name}</Text>
                    {roommate.online && <View style={styles.onlineDot} />}
                    {roommate.verified && (
                      <View style={styles.verifiedBadge}>
                        <Ionicons name="checkmark" size={10} color="#FFFFFF" />
                      </View>
                    )}
                  </View>
                  <View style={styles.subtitleRow}>
                    <Text style={styles.subtitle}>{roommate.age} years old</Text>
                    <Text style={styles.subtitle}>•</Text>
                    <Text style={styles.subtitle}>{roommate.distance}</Text>
                    <Text style={styles.subtitle}>•</Text>
                    <View style={styles.ratingRow}>
                      <Ionicons name="star" size={12} color="#F59E0B" />
                      <Text style={styles.rating}>{roommate.rating}</Text>
                    </View>
                  </View>
                </View>
                <View style={styles.headerActions}>
                  <TouchableOpacity
                    style={styles.favoriteBtn}
                    onPress={() => toggleFavorite(roommate.id)}
                  >
                    <Ionicons 
                      name={favorites.has(roommate.id) ? "heart" : "heart-outline"} 
                      size={20} 
                      color={favorites.has(roommate.id) ? "#EF4444" : "#9CA3AF"} 
                    />
                  </TouchableOpacity>
                  <View style={styles.availableBadge}>
                    <Text style={styles.availableText}>Available</Text>
                  </View>
                </View>
              </View>

              {/* Bio */}
              <Text style={styles.bio}>{roommate.bio}</Text>

              {/* Tags */}
              <View style={styles.tags}>
                {roommate.tags.map((tag) => {
                  const tagStyle = getTagStyle(tag);
                  return (
                    <View key={tag} style={[styles.tag, { backgroundColor: tagStyle.backgroundColor }]}>
                      <Text style={[styles.tagText, { color: tagStyle.color }]}>{tag}</Text>
                    </View>
                  );
                })}
              </View>

              {/* Footer */}
              <View style={styles.cardFooter}>
                <Text style={styles.price}>${roommate.price}/month</Text>
                <View style={styles.actionButtons}>
                  <TouchableOpacity style={styles.viewBtn}>
                    <Ionicons name="eye" size={16} color="#FFFFFF" />
                    <Text style={styles.viewBtnText}>View</Text>
                  </TouchableOpacity>
                  <TouchableOpacity style={styles.messageBtn}>
                    <Ionicons name="chatbubble" size={16} color="#FFFFFF" />
                    <Text style={styles.messageBtnText}>Message</Text>
                  </TouchableOpacity>
                </View>
              </View>
            </View>
          ))}
        </ScrollView>
      </View>
    </SafeAreaView>
  );
};

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#1F2937',
  },
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 16,
    paddingVertical: 12,
    backgroundColor: '#1F2937',
    borderBottomWidth: 1,
    borderBottomColor: '#374151',
  },
  logo: {
    color: '#FFFFFF',
    fontSize: 20,
    fontWeight: 'bold',
  },
  filterBtn: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#374151',
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 20,
    gap: 6,
  },
  filterText: {
    color: '#FFFFFF',
    fontSize: 14,
  },
  searchRow: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#374151',
    borderRadius: 12,
    margin: 12,
    paddingHorizontal: 12,
    gap: 8,
  },
  searchInput: {
    flex: 1,
    color: '#FFFFFF',
    paddingVertical: 12,
    fontSize: 15,
  },
  mapToggle: {
    backgroundColor: '#111827',
    paddingVertical: 8,
    paddingHorizontal: 12,
    borderRadius: 8,
  },
  mapToggleText: {
    color: '#FFFFFF',
    fontSize: 13,
  },
  advancedFilters: {
    backgroundColor: '#374151',
    marginHorizontal: 12,
    marginBottom: 12,
    padding: 16,
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#4B5563',
  },
  filterRow: {
    flexDirection: 'row',
    gap: 16,
  },
  filterGroup: {
    flex: 1,
  },
  filterLabel: {
    color: '#FFFFFF',
    fontSize: 14,
    fontWeight: '500',
    marginBottom: 8,
  },
  priceInputs: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
  },
  priceInput: {
    flex: 1,
    backgroundColor: '#111827',
    color: '#FFFFFF',
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 8,
    fontSize: 14,
  },
  priceSeparator: {
    color: '#9CA3AF',
  },
  selectInput: {
    flexDirection: 'row',
    alignItems: 'center',
    justifyContent: 'space-between',
    backgroundColor: '#111827',
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 8,
  },
  selectText: {
    color: '#FFFFFF',
    fontSize: 14,
  },
  mapArea: {
    flex: 1,
    backgroundColor: '#0F172A',
    position: 'relative',
  },
  mapPlaceholder: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },
  mapText: {
    color: '#9CA3AF',
    fontSize: 16,
    marginTop: 8,
  },
  mapSubtext: {
    color: '#6B7280',
    fontSize: 14,
    marginTop: 4,
  },
  mapPin: {
    position: 'absolute',
    width: 24,
    height: 24,
    backgroundColor: '#3B82F6',
    borderRadius: 12,
    borderWidth: 2,
    borderColor: '#FFFFFF',
  },
  zoomControls: {
    position: 'absolute',
    right: 16,
    bottom: 16,
    gap: 8,
  },
  zoomBtn: {
    backgroundColor: '#374151',
    width: 44,
    height: 44,
    borderRadius: 22,
    justifyContent: 'center',
    alignItems: 'center',
  },
  bottomPanel: {
    backgroundColor: '#FFFFFF',
    borderTopLeftRadius: 20,
    borderTopRightRadius: 20,
    maxHeight: height * 0.4,
  },
  filtersSection: {
    padding: 16,
    borderBottomWidth: 1,
    borderBottomColor: '#E5E7EB',
  },
  filtersRow: {
    marginBottom: 12,
  },
  filterChip: {
    paddingHorizontal: 16,
    paddingVertical: 8,
    backgroundColor: '#F3F4F6',
    borderRadius: 20,
    marginRight: 8,
  },
  filterChipActive: {
    backgroundColor: '#3B82F6',
  },
  filterChipText: {
    fontSize: 13,
    color: '#111827',
    fontWeight: '500',
  },
  filterChipTextActive: {
    color: '#FFFFFF',
  },
  resultRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  resultCount: {
    color: '#6B7280',
    fontSize: 14,
  },
  sortButton: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 4,
  },
  sortText: {
    color: '#3B82F6',
    fontSize: 14,
  },
  cardsList: {
    flex: 0,
    paddingHorizontal: 16,
  },
  card: {
    backgroundColor: '#1F2937',
    borderRadius: 16,
    padding: 16,
    marginBottom: 12,
  },
  cardHeader: {
    flexDirection: 'row',
    alignItems: 'center',
    marginBottom: 12,
    gap: 12,
  },
  avatar: {
    width: 48,
    height: 48,
    backgroundColor: '#3B82F6',
    borderRadius: 24,
    justifyContent: 'center',
    alignItems: 'center',
  },
  avatarText: {
    color: '#FFFFFF',
    fontSize: 18,
    fontWeight: 'bold',
  },
  userInfo: {
    flex: 1,
  },
  nameRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 8,
    marginBottom: 4,
  },
  name: {
    color: '#FFFFFF',
    fontSize: 16,
    fontWeight: '600',
  },
  onlineDot: {
    width: 8,
    height: 8,
    backgroundColor: '#22C55E',
    borderRadius: 4,
  },
  verifiedBadge: {
    width: 16,
    height: 16,
    backgroundColor: '#3B82F6',
    borderRadius: 8,
    justifyContent: 'center',
    alignItems: 'center',
  },
  subtitleRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 6,
  },
  subtitle: {
    color: '#9CA3AF',
    fontSize: 13,
  },
  ratingRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 2,
  },
  rating: {
    color: '#9CA3AF',
    fontSize: 13,
  },
  headerActions: {
    alignItems: 'center',
    gap: 8,
  },
  favoriteBtn: {
    padding: 4,
  },
  availableBadge: {
    backgroundColor: '#3B82F6',
    paddingHorizontal: 8,
    paddingVertical: 4,
    borderRadius: 12,
  },
  availableText: {
    color: '#FFFFFF',
    fontSize: 11,
    fontWeight: '500',
  },
  bio: {
    color: '#D1D5DB',
    fontSize: 14,
    lineHeight: 20,
    marginBottom: 12,
  },
  tags: {
    flexDirection: 'row',
    flexWrap: 'wrap',
    gap: 8,
    marginBottom: 16,
  },
  tag: {
    paddingHorizontal: 12,
    paddingVertical: 6,
    borderRadius: 16,
  },
  tagText: {
    fontSize: 12,
    fontWeight: '500',
  },
  cardFooter: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  price: {
    color: '#22C55E',
    fontSize: 18,
    fontWeight: 'bold',
  },
  actionButtons: {
    flexDirection: 'row',
    gap: 8,
  },
  viewBtn: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#374151',
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 8,
    gap: 4,
  },
  viewBtnText: {
    color: '#FFFFFF',
    fontSize: 13,
    fontWeight: '500',
  },
  messageBtn: {
    flexDirection: 'row',
    alignItems: 'center',
    backgroundColor: '#3B82F6',
    paddingHorizontal: 12,
    paddingVertical: 8,
    borderRadius: 8,
    gap: 4,
  },
  messageBtnText: {
    color: '#FFFFFF',
    fontSize: 13,
    fontWeight: '500',
  },
});

export default RoommateMapScreen;