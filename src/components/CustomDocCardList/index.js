import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import {
  useCurrentSidebarCategory,
  filterDocCardListItems,
} from '@docusaurus/theme-common';
import DocCardList from '@theme/DocCardList';
import DocCard from '@theme/DocCard';
import { useLocation } from '@docusaurus/router';

// Custom SidebarItem component that tracks expanded state
function CustomSidebarItem({ item, ...props }) {
  const { href, label, type } = item;
  const [isExpanded, setIsExpanded] = useState(false);

  // Set default collapsed state for category items
  useEffect(() => {
    if (type === 'category') {
      setIsExpanded(false); // Default to collapsed
    }
  }, [type]);

  if (type === 'category') {
    const { items, collapsed, collapsible } = item;
    const filteredItems = filterDocCardListItems(items);

    if (filteredItems.length === 0) {
      return null;
    }

    return (
      <div
        className={clsx('menu__list-item', {
          'menu__list-item--collapsed': collapsed && isExpanded,
        })}
      >
        <button
          className={clsx(
            'menu__caret',
            !collapsible && 'menu__caret--disabled',
          )}
          aria-label={collapsible ? `Toggle the submenu for "${label}"` : undefined}
          onClick={(e) => {
            e.preventDefault();
            if (collapsible) {
              setIsExpanded(!isExpanded);
            }
          }}
          aria-expanded={isExpanded}
          tabIndex={collapsible ? 0 : -1}
        />
        <a
          className={clsx('menu__link', {
            'menu__link--sublist': collapsible,
            'menu__link--sublist-caret': collapsible && !collapsed,
          })}
          {...props}
          href={collapsible ? href : '#'}
          onClick={(e) => {
            if (collapsible && href) {
              // If there's a link and it's collapsible, allow navigation
              return;
            } else if (collapsible) {
              // If it's just collapsible without a link, prevent default and toggle
              e.preventDefault();
              setIsExpanded(!isExpanded);
            }
          }}
        >
          {label}
        </a>
        <ul
          className="menu__list"
          role="list"
          style={{
            height: isExpanded ? 'auto' : 0,
            overflow: isExpanded ? 'visible' : 'hidden',
            opacity: isExpanded ? 1 : 0,
          }}
        >
          {filteredItems.map((childItem) => (
            <li className="menu__list-item" key={childItem.docId ?? childItem.href}>
              <CustomSidebarItem
                item={childItem}
                {...props}
              />
            </li>
          ))}
        </ul>
      </div>
    );
  }

  return (
    <li className="menu__list-item">
      <DocCard item={item} />
    </li>
  );
}

// Custom DocCardList component that uses our custom sidebar item
export default function CustomDocCardList(props) {
  const { items } = props;
  const filteredItems = filterDocCardListItems(items);

  return (
    <section className="row">
      {filteredItems.map((item) => (
        <article key={item.docId ?? item.href} className="col col--12">
          <CustomSidebarItem item={item} />
        </article>
      ))}
    </section>
  );
}