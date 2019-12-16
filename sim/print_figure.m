function print_figure(fig, name)
%PRINT_FIGURE Summary of this function goes here
%   Detailed explanation goes here
% add axis labes and legend
% axis tight

% set text properties

fig.Units = 'points';
% children = fig.Children;
% set(findall(fig,'type','text'),'FontSize',10)

% if fig.Number == 6
%     children = fig.Children(2);
% end
% 
% if strcmp(class(children),  'matlab.graphics.layout.TiledChartLayout')
%     c = children.Children;
% else
%     c = children;
% end
% 
% if fig.Number ~= 2 && fig.Number ~=3
%     set(c, 'FontSize', 10);
% else
%     set(children.XLabel, 'FontSize', 16);
%     set(children.YLabel, 'FontSize', 16);
%     set(c, 'FontSize', 16);
% end
% set(findall(fig,'type','text'),'FontSize',10)
% remove unnecessary white space
% set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))

% set(fig,'renderer','Painters')

% export to eps
% fig.PaperPositionMode   = 'auto';

output_folder = fullfile(pwd, 'figures');
file_name = fullfile(output_folder, strcat(name, '.pdf'));
% print(file_name, '-depsc2', '-painters', '-tiff')
% set(fig, 'Renderer', 'opengl')
set(gcf,'color','w');

pos = get(fig,'Position');
unit = get(fig, 'Units');
set(fig,'PaperPositionMode','Auto', 'PaperType', 'b5', 'PaperUnits',unit,'PaperSize',[pos(3), pos(4)], 'PaperPosition', pos) %
print(file_name, '-dpdf', '-painters')
%export_fig(fig, file_name, '-pdf', '-painters')
end

