import React from 'react';
import DocSidebarItemCategory from '@theme/DocSidebarItem/Category';
import DocSidebarItemLink from '@theme/DocSidebarItem/Link';

// Custom sidebar item component that passes through to the default implementations
// The actual collapsing behavior will be controlled via the sidebar configuration
export default function DocSidebarItem({ item, ...props }) {
  if (item.type === 'category') {
    // For category items, ensure they are collapsed by default
    // We set the collapsed property to true if it's not explicitly set to false
    const modifiedItem = {
      ...item,
      collapsed: item.collapsed !== false, // Default to true (collapsed) unless explicitly set to false
    };

    return <DocSidebarItemCategory {...props} item={modifiedItem} />;
  }

  return <DocSidebarItemLink {...props} item={item} />;
}