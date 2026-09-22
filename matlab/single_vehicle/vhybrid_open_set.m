classdef vhybrid_open_set < handle
    %VHYBRID_OPEN_SET 用最小 f_cost 选择待扩展节点。

    properties (Access = private)
        nodes_;
    end

    methods
        function obj = vhybrid_open_set()
            % 构造空 Open 集合。
            obj.nodes_ = repmat(vhybrid_node(), 0, 1);
        end

        function push(obj, node)
            %PUSH 将候选节点放入 Open 集合。
            obj.nodes_(end+1,1) = node;
        end

        function [node, valid] = pop_min(obj)
            %POP_MIN 取出 f_cost 最小的节点。
            if isempty(obj.nodes_)
                node = vhybrid_node();
                valid = false;
                return;
            end
            [~, local_index] = min([obj.nodes_.f_cost]);
            node = obj.nodes_(local_index);
            obj.nodes_(local_index) = [];
            valid = true;
        end

        function flag = is_empty(obj)
            %IS_EMPTY 判断 Open 集合是否为空。
            flag = isempty(obj.nodes_);
        end

        function n = count(obj)
            %COUNT 返回 Open 集合当前节点数量。
            n = numel(obj.nodes_);
        end
    end
end
