from pygments import highlight
from pygments.lexers import JsonLexer
from pygments.formatters import Terminal256Formatter
import json

# Sample data structure
data = {
    "name": "Alice",
    "age": 28,
    "skills": ["Python", "JavaScript", "SQL"],
    "projects": {"open_source": True, "count": 4},
}

# Convert to formatted JSON string
json_str = json.dumps(data, indent=4)

# Create formatter for terminal output
formatter = Terminal256Formatter(
    style="monokai",  # Color scheme (try "solarized", "fruity", or "native")
)

# Colorize and print directly to terminal
highlighted_json = highlight(json_str, JsonLexer(), formatter)
print(highlighted_json)
