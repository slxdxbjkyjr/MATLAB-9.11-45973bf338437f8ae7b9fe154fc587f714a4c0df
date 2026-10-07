classdef vhybrid_open_set < handle
    %VHYBRID_OPEN_SET 用最小 f_cost 选择待扩展节点。

    properties (Access = private)
        nodes_;
        best_g_;
    end

    methods
        function obj = vhybrid_open_set()
            % 构造空 Open 集合。
            obj.nodes_ = repmat(vhybrid_node(), 0, 1);
            obj.best_g_ = containers.Map('KeyType', 'char', 'ValueType', 'double');
        end

        function push(obj, node)
            %PUSH 将候选节点放入 Open 集合。
            key = obj.key(node);
            if isKey(obj.best_g_, key) && obj.best_g_(key) <= node.g_cost
                return;
            end
            obj.best_g_(key) = node.g_cost;
            % 删除末节点后空列可能成为 1x0；numel 避免生成空字段节点。
            obj.nodes_(numel(obj.nodes_)+1,1) = node;
        end

        function accepted = push_or_update(obj, node)
            %PUSH_OR_UPDATE 插入新状态或用更小 g 值替换旧候选。
            key = obj.key(node);
            accepted = ~(isKey(obj.best_g_, key) && obj.best_g_(key) <= node.g_cost);
            if ~accepted
                return;
            end
            obj.best_g_(key) = node.g_cost;
            obj.nodes_(numel(obj.nodes_)+1,1) = node;
        end

        function [node, valid] = pop_min(obj)
            %POP_MIN 取出 f_cost 最小的节点。
            if isempty(obj.nodes_)
                node = vhybrid_node();
                valid = false;
                return;
            end
            [~, local_index] = min([obj.nodes_.f_cost]);
            if isempty(local_index)
                node = vhybrid_node();
                valid = false;
                return;
            end
            node = obj.nodes_(local_index);
            obj.nodes_(local_index) = [];
            obj.nodes_ = obj.nodes_(:);
            key = obj.key(node);
            if isKey(obj.best_g_, key) && node.g_cost > obj.best_g_(key) + 1e-12
                [node, valid] = obj.pop_min();
                return;
            end
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

    methods (Access = private)
        function key = key(~, node)
            %KEY 使用五维离散索引区分位置、航向、速度和时间。
            key = sprintf('%d_%d_%d_%d_%d', node.x_index, node.y_index, ...
                node.yaw_index, node.velocity_index, node.time_index);
        end
    end
end
