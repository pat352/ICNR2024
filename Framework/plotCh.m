function plotCh(data, matSize, fs, varargin)
%PLOTCH plots all channels in data [channels x samples] with an offset
if nargin < 4
    offset = max(abs(data(:)))/5;
else
    offset = varargin{1};
end

% ch = cell(1, size(data, 1));
ch = cell(0);
ch{1} = 'ch 1';
n = 1;

t = linspace(0, size(data,2)/double(fs), size(data,2));

cmap = parula(size(data, 1)); cmap = flipud(cmap);
for i = 1: size(data, 1)
    
    plot(t, data(i,:) + offset*(i-1), 'Color', cmap(i,:)), hold on
    
    if rem(i,matSize) == 0
        n = n +1;
        ch{1,n} = sprintf('ch %g', i); 
    end
end

% xlim([1, size(data, 2)]);
% yticks(0:offset:offset*(i-1));

xlim([0, max(t)]);
yticks([0, offset*(matSize-1):offset*matSize:offset*i]);%(i-1));
% yticks(offset*matSize:offset*matSize:offset*(i-1));
yticklabels(ch);
xlabel('Time [s]');
ylabel('Channel number');
set(gca,'FontSize',20);