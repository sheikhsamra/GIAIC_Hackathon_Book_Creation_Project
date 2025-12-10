// Simple client-side markdown to HTML converter
// This is a basic implementation that handles common markdown elements
// For a production app, you'd want to use a proper library like marked or markdown-it
export function markdownToHtml(markdown) {
  if (!markdown) return '';

  // Process the markdown content step by step
  let html = markdown;

  // Handle headers
  html = html.replace(/^### (.*$)/gim, '<h3>$1</h3>');
  html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
  html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

  // Handle bold and italic
  html = html.replace(/\*\*(.*?)\*\*/gim, '<strong>$1</strong>');
  html = html.replace(/\*(.*?)\*/gim, '<em>$1</em>');

  // Handle links
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/gim, '<a href="$2">$1</a>');

  // Handle images
  html = html.replace(/!\[([^\]]*)\]\(([^)]+)\)/gim, '<img src="$2" alt="$1" />');

  // Handle unordered lists
  html = html.replace(/^\s*\*\s(.+)$/gim, '<li>$1</li>');
  html = html.replace(/(<li>.*<\/li>)+/gim, '<ul>$&</ul>');

  // Handle paragraphs
  html = html.replace(/^\s*(\w.+)/gim, '<p>$1</p>');

  // Handle line breaks
  html = html.split('\n').join(' ');

  // Clean up multiple spaces
  html = html.replace(/\s+/g, ' ');

  return html;
}

// Alternative function using DOMPurify to sanitize the output (if available)
export function safeMarkdownToHtml(markdown) {
  const html = markdownToHtml(markdown);

  // If DOMPurify is available, use it to sanitize the HTML
  if (typeof window !== 'undefined' && window.DOMPurify) {
    return window.DOMPurify.sanitize(html);
  }

  return html;
}