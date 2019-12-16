function print_figure(fig, name)

fig.Units = 'points';

output_folder = fullfile(pwd, 'figures');
file_name = fullfile(output_folder, strcat(name, '.pdf'));

set(gcf,'color','w');

pos = get(fig,'Position');
unit = get(fig, 'Units');
set(fig,'PaperPositionMode','Auto', 'PaperType', 'b5', 'PaperUnits',unit,'PaperSize',[pos(3), pos(4)], 'PaperPosition', pos) %
print(file_name, '-dpdf', '-painters')

end

