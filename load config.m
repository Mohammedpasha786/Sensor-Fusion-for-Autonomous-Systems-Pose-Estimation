function cfg = loadConfig(yamlFile)
% LOADCONFIG  Lightweight YAML parser for project configuration files.
%
%   cfg = loadConfig(yamlFile)
%
%   Supports:
%     - Nested mappings (indentation-based)
%     - Scalars: numbers, strings, booleans
%     - Inline lists: [a, b, c]
%     - Comments (#) and blank lines
%
%   This is a minimal parser sufficient for the flat/nested numeric
%   configs used in this project. For full YAML 1.2 support, use
%   MATLAB's built-in yaml functions (R2024a+) or a third-party library
%   such as YAMLMatlab.
%
%   Input:
%     yamlFile - path to .yaml/.yml file
%
%   Output:
%     cfg      - nested struct mirroring the YAML hierarchy

    assert(isfile(yamlFile), 'Config file not found: %s', yamlFile);

    % Prefer built-in reader if available (R2024a+)
    if exist('readyaml', 'file') == 2 || exist('readyaml', 'builtin') == 5
        cfg = readyaml(yamlFile);
        return;
    end

    %% ── Fallback: minimal custom parser ─────────────────────────────────
    lines = readlines(yamlFile);
    cfg = struct();
    stack = {cfg};        % stack of struct references (by path)
    pathStack = {{}};     % stack of field-name paths
    indentStack = [-1];   % indentation levels

    root = struct();
    pathMap = {{} root};   % path -> placeholder (unused, simple approach)

    % Use a simple recursive-descent based on indentation
    [root, ~] = parseBlock(lines, 1, -1);
    cfg = root;
end


function [result, nextIdx] = parseBlock(lines, startIdx, parentIndent)
% Recursively parse a block of YAML lines into a struct.
    result = struct();
    i = startIdx;
    n = numel(lines);

    while i <= n
        rawLine = lines(i);
        line = char(rawLine);

        % Skip blanks/comments
        trimmed = strtrim(line);
        if isempty(trimmed) || startsWith(trimmed, '#')
            i = i + 1;
            continue;
        end

        % Determine indentation
        indent = length(line) - length(strtrim(line));
        % erase leading spaces properly (count spaces only)
        nSpaces = 0;
        for c = 1:length(line)
            if line(c) == ' '
                nSpaces = nSpaces + 1;
            else
                break;
            end
        end
        indent = nSpaces;

        if indent <= parentIndent
            % This line belongs to parent — stop
            nextIdx = i;
            return;
        end

        % Remove inline comment
        trimmed = regexprep(trimmed, '\s+#.*$', '');

        % Split "key: value"
        colonIdx = find(trimmed == ':', 1);
        assert(~isempty(colonIdx), 'Invalid YAML line: %s', trimmed);

        key = strtrim(trimmed(1:colonIdx-1));
        valueStr = strtrim(trimmed(colonIdx+1:end));

        fieldName = sanitizeFieldName(key);

        if isempty(valueStr)
            % Nested block follows
            [childStruct, nextI] = parseBlock(lines, i+1, indent);
            result.(fieldName) = childStruct;
            i = nextI;
        else
            result.(fieldName) = parseScalarOrList(valueStr);
            i = i + 1;
        end
    end

    nextIdx = i;
end


function fieldName = sanitizeFieldName(key)
% Convert YAML key to valid MATLAB struct field name.
    fieldName = regexprep(key, '[^a-zA-Z0-9_]', '_');
    if ~isempty(fieldName) && ~isletter(fieldName(1))
        fieldName = ['f_' fieldName];
    end
end


function val = parseScalarOrList(valueStr)
% Parse a YAML scalar or inline list into a MATLAB value.

    valueStr = strtrim(valueStr);

    % Remove surrounding quotes
    if (startsWith(valueStr, '"') && endsWith(valueStr, '"')) || ...
       (startsWith(valueStr, "'") && endsWith(valueStr, "'"))
        val = valueStr(2:end-1);
        return;
    end

    % Inline list: [a, b, c]
    if startsWith(valueStr, '[') && endsWith(valueStr, ']')
        inner = valueStr(2:end-1);
        if isempty(strtrim(inner))
            val = [];
            return;
        end
        items = strsplit(inner, ',');
        items = strtrim(items);
        numericVals = str2double(items);
        if all(~isnan(numericVals))
            val = numericVals;
        else
            val = items;  % cell array of strings
        end
        return;
    end

    % Boolean
    if strcmpi(valueStr, 'true')
        val = true; return;
    elseif strcmpi(valueStr, 'false')
        val = false; return;
    end

    % Numeric
    numVal = str2double(valueStr);
    if ~isnan(numVal)
        val = numVal;
        return;
    end

    % Fallback: string
    val = valueStr;
end
