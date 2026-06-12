package gds.cloud.module.antidrone.utils.radar.calibration.util;

import java.util.*;

/**
 * 简易 JSON 解析器（无第三方依赖）
 *
 * 支持：对象、数组、字符串、数字、布尔值、null
 * 限制：不支持转义字符、不验证格式
 */
public class SimpleJsonParser {

    private final String json;
    private int pos;

    private SimpleJsonParser(String json) {
        this.json = json;
        this.pos = 0;
    }

    public static Map<String, Object> parse(String json) {
        SimpleJsonParser parser = new SimpleJsonParser(json.trim());
        Object result = parser.parseValue();
        @SuppressWarnings("unchecked")
        Map<String, Object> map = (Map<String, Object>) result;
        return map;
    }

    private Object parseValue() {
        skipWhitespace();
        if (pos >= json.length()) return null;

        char c = json.charAt(pos);

        switch (c) {
            case '{': return parseObject();
            case '[': return parseArray();
            case '"': return parseString();
            case 't': case 'f': return parseBoolean();
            case 'n': return parseNull();
            default: return parseNumber();
        }
    }

    private Map<String, Object> parseObject() {
        Map<String, Object> map = new LinkedHashMap<>();
        pos++; // skip '{'
        skipWhitespace();

        if (peek() == '}') {
            pos++;
            return map;
        }

        while (true) {
            skipWhitespace();
            String key = parseString();
            skipWhitespace();
            if (peek() == ':') pos++;
            Object value = parseValue();
            map.put(key, value);

            skipWhitespace();
            char c = peek();
            if (c == ',') {
                pos++;
            } else if (c == '}') {
                pos++;
                break;
            }
        }

        return map;
    }

    private List<Object> parseArray() {
        List<Object> list = new ArrayList<>();
        pos++; // skip '['
        skipWhitespace();

        if (peek() == ']') {
            pos++;
            return list;
        }

        while (true) {
            Object value = parseValue();
            list.add(value);

            skipWhitespace();
            char c = peek();
            if (c == ',') {
                pos++;
            } else if (c == ']') {
                pos++;
                break;
            }
        }

        return list;
    }

    private String parseString() {
        pos++; // skip opening quote
        StringBuilder sb = new StringBuilder();

        while (pos < json.length()) {
            char c = json.charAt(pos);
            if (c == '"') {
                pos++;
                break;
            }
            sb.append(c);
            pos++;
        }

        return sb.toString();
    }

    private Boolean parseBoolean() {
        if (json.startsWith("true", pos)) {
            pos += 4;
            return true;
        } else if (json.startsWith("false", pos)) {
            pos += 5;
            return false;
        }
        return null;
    }

    private Object parseNull() {
        if (json.startsWith("null", pos)) {
            pos += 4;
        }
        return null;
    }

    private Number parseNumber() {
        int start = pos;
        boolean isDouble = false;

        while (pos < json.length()) {
            char c = json.charAt(pos);
            if (c == '-' || c == '+' || c == 'e' || c == 'E' || c == '.') {
                if (c == '.') isDouble = true;
                pos++;
            } else if (c >= '0' && c <= '9') {
                pos++;
            } else {
                break;
            }
        }

        String numStr = json.substring(start, pos);
        try {
            if (isDouble) {
                return Double.parseDouble(numStr);
            } else {
                return Long.parseLong(numStr);
            }
        } catch (NumberFormatException e) {
            return Double.parseDouble(numStr);
        }
    }

    private void skipWhitespace() {
        while (pos < json.length()) {
            char c = json.charAt(pos);
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                pos++;
            } else {
                break;
            }
        }
    }

    private char peek() {
        return pos < json.length() ? json.charAt(pos) : '\0';
    }
}
